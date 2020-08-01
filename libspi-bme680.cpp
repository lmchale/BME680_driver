
#include <vector>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "libspi-bme680.h"

namespace fs = std::filesystem;

// Static Member Variables:
std::map<unsigned int, spiDev*> bme680::devMapper_;


/// SPIDEV ////////////////////////////////////////////////////////////////////
spiDev::spiDev(fs::path device) {
	if (!fs::is_character_file(device))
		throw std::runtime_error("Path is not a character device");

	// Attempt to open device:
	//fd_ = open(device.c_str(), O_DIRECT | O_RDWR);
	fd_ = open(device.c_str(), O_RDWR);
	if (fd_ == -1) {
		perror("open");
		throw std::runtime_error("Failed to open character device");
	}

	// Configure some spidev paramters:
	uint32_t speed = 1000000;	// 1 MHz
	auto ret = ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		perror("ioctl");
	ret = ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		perror("ioctl");
}


spiDev::~spiDev() {
	close(fd_);
}


int spiDev::read(uint8_t reg, uint8_t* data, size_t bytes, uint16_t d_us) {
	/*
	 * Data on the bus should be like
	 * |---------------------+--------------+-------------|
	 * | MOSI                | MISO         | Chip Select |
	 * |---------------------+--------------|-------------|
	 * | (don't care)        | (don't care) | HIGH        |
	 * | (reg_addr)          | (don't care) | LOW         |
	 * | (reg_data[0])       | (don't care) | LOW         |
	 * | (....)              | (....)       | LOW         |
	 * | (reg_data[len - 1]) | (don't care) | LOW         |
	 * | (don't care)        | (don't care) | HIGH        |
	 * |---------------------+--------------|-------------|
	 */
#ifdef DEBUG
	printf("read  %iB: (%s reg %02x) <-", bytes,
		(reg&0x80) == 0x80 ? "R" : "W" , reg&0x7F);
#endif

	std::vector<uint8_t> buf(bytes);
	spi_ioc_transfer xfer[2] = {};

	// Write address:
	xfer[0].tx_buf = reinterpret_cast<Kptr>(&reg);
	xfer[0].len = 1;
	xfer[0].cs_change = false;
	xfer[0].delay_usecs = d_us;

	// Read data:
	xfer[1].rx_buf = reinterpret_cast<Kptr>(buf.data());
	xfer[1].len = buf.size();

	// Send transfer bundle:
	int status = ioctl(fd_, SPI_IOC_MESSAGE(2), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
		throw std::logic_error("SPI_IOC_MESSAGE error during read");
	}

#ifdef DEBUG
	for (int i = 0; i < bytes; i++) {
		printf(" %02x", buf[i]);
	}
	putchar('\n');
#endif

	memcpy(data, buf.data(), bytes);
	return 0;
}


int spiDev::write(uint8_t reg, uint8_t* data, size_t bytes) {
	/*
	 * Data on the bus should be like
	 * |---------------------+--------------+-------------|
	 * | MOSI                | MISO         | Chip Select |
	 * |---------------------+--------------|-------------|
	 * | (don't care)        | (don't care) | HIGH        |
	 * | (reg_addr)          | (don't care) | LOW         |
	 * | (reg_data[0])       | (don't care) | LOW         |
	 * | (....)              | (....)       | LOW         |
	 * | (reg_data[len - 1]) | (don't care) | LOW         |
	 * | (don't care)        | (don't care) | HIGH        |
	 * |---------------------+--------------|-------------|
	 */
#ifdef DEBUG
	printf("write %iB: (%s reg %02x) ->", bytes,
		(reg&0x80) == 0x80 ? "R" : "W" , reg&0x7F);
	for (int i = 0; i < bytes; i++) {
		printf(" %02x", data[i]);
	}
	putchar('\n');
#endif

	std::vector<uint8_t> buf(1+bytes);
	buf[0] = reg;
	memcpy(buf.data()+1, data, bytes);

	spi_ioc_transfer xfer[1] = {};

	// Write address immediately followed by data:
	xfer[0].tx_buf = reinterpret_cast<Kptr>(buf.data());
	xfer[0].len = buf.size();

	// Send transfer bundle:
	int status = ioctl(fd_, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
		throw std::logic_error("SPI_IOC_MESSAGE error during write");
	}
	return 0;
}


/// BME680 ////////////////////////////////////////////////////////////////////
bme680::bme680(fs::path device) : dev_(device) {
#ifdef DEBUG
	std::cout << __PRETTY_FUNCTION__ << std::endl;
#endif
	// Ensure spi device not already oppened and mapped:
	std::string idStr = device.extension().string();
	idStr.erase(idStr.begin());
	unsigned int id = std::stoul(idStr);
	if (devMapper_.find(id) != devMapper_.end()) {
		throw std::logic_error("Tried to open character device more than once");
	}
	
	// Register sucessfully opened device for callbacks:
	devMapper_.emplace(id, &dev_);

	/* You may assign a chip select identifier to be passed to callbacks later */
	sensor_.dev_id = id;
	sensor_.intf = BME680_SPI_INTF;
	sensor_.read = user_spi_read;
	sensor_.write = user_spi_write;
	sensor_.delay_ms = user_delay_ms;
	/* amb_temp can be set to 25 prior to configuring the gas sensor 
	 * or by performing a few temperature readings without operating the gas sensor.
	 */
	sensor_.amb_temp = 25;
	
	int8_t rslt = bme680_init(&sensor_);
	if (rslt != BME680_OK) {
		std::cerr << "bme680_init returned " << int(rslt) << std::endl;
		throw std::runtime_error("bme680_init failed");
	}
}


bme680::~bme680() {
	// Unregister device from callbacks before closing:
	for (const auto [key, value] : devMapper_) {
		if (value == &dev_) {
			devMapper_.erase(key);
			break;
		}
	}
}


spiDev& bme680::lookup(uint8_t dev_id) {
	return *devMapper_.at(dev_id);
}


void bme680::configure() {
#ifdef DEBUG
	std::cout << __PRETTY_FUNCTION__ << std::endl;
#endif
	// Set the temperature, pressure and humidity settings
	sensor_.tph_sett.os_hum = BME680_OS_2X;
	sensor_.tph_sett.os_pres = BME680_OS_4X;
	sensor_.tph_sett.os_temp = BME680_OS_8X;
	sensor_.tph_sett.filter = BME680_FILTER_SIZE_3;
	
	// Set the remaining gas sensor settings and link the heating profile
	sensor_.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
	// Create a ramp heat waveform in 3 steps
	sensor_.gas_sett.heatr_temp = 320;	// degree Celsius
	sensor_.gas_sett.heatr_dur = 150;	// milliseconds
	
	// Select the power mode
	// Must be set before writing the sensor configuration
	sensor_.power_mode = BME680_FORCED_MODE; 
	
	// Set the required sensor settings needed
	uint8_t set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | 
					BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;
	
	// Set the desired sensor configuration
	auto rslt = bme680_set_sensor_settings(set_required_settings, &sensor_);
}


Measurement bme680::measure() {
#ifdef DEBUG
	std::cout << __PRETTY_FUNCTION__ << std::endl;
#endif
	// Perform measurement
	auto rslt = bme680_set_sensor_mode(&sensor_);

	// Delay till the measurement is ready
	uint16_t meas_period;
	bme680_get_profile_dur(&meas_period, &sensor_);
	user_delay_ms(meas_period);
	
	struct bme680_field_data data;
	rslt = bme680_get_sensor_data(&data, &sensor_);
	
#ifdef DEBUG
	printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH",
		data.temperature / 100.0f,
	  data.pressure / 100.0f,
		data.humidity / 1000.0f);
	
	// Avoid using measurements from an unstable heating setup
	if (data.status & BME680_GASM_VALID_MSK) {
		printf(", G: %d ohms", data.gas_resistance);
	}
	putchar('\n');
#endif

	Measurement m = {
		data.temperature/100.0f,
		data.pressure/100.0f,
		data.humidity/1000.f,
		(data.status & BME680_GASM_VALID_MSK) ? std::optional<int>(data.gas_resistance) : std::nullopt
	};
	return m;
}


/// MEASUREMENT ///////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& os, const Measurement& m) {
	os << m.temp_C << ',' 
	   << m.pressure_hPa << ','
	   << m.relHumidity;
	if (m.gas_ohms) {
		os << ',' << *(m.gas_ohms);
	}
	return os;
}


/// BME680 CALLBACK INTERFACE /////////////////////////////////////////////////
/*
 * Return control or wait,
 * for a period amount of milliseconds
 */
void user_delay_ms(uint32_t ms) {
#ifdef DEBUG
	std::cout << __PRETTY_FUNCTION__ << std::endl;
#endif
	std::this_thread::sleep_for( std::chrono::milliseconds(ms) );
}


/*
 * The parameter dev_id can be used as a variable to select which Chip Select pin has
 * to be set low to activate the relevant device on the SPI bus
 */
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, 
                     uint8_t *reg_data, uint16_t len) {
	if (len == 0) {
		std::cerr << "Warning: spi callback to read 0 bytes from reg " << reg_addr << std::endl;
		return 0;
	}

	spiDev& spi = bme680::lookup(dev_id);
	return spi.read(reg_addr, reg_data, len);
}


/*
 * The parameter dev_id can be used as a variable to select which Chip Select pin has
 * to be set low to activate the relevant device on the SPI bus
 */
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr,
                      uint8_t *reg_data, uint16_t len) {
	if (len == 0) {
		std::cerr << "Warning: spi callback to write 0 bytes from reg " << reg_addr << std::endl;
		return 0;
	}

	spiDev& spi = bme680::lookup(dev_id);
	return spi.write(reg_addr, reg_data, len);
}


