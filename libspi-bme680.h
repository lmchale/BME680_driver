#ifndef LIBSPI_BME680_H
#define LIBSPI_BME680_H

#include <map>
#include <optional>

#include "bme680.h"

namespace fs = std::filesystem;

extern "C" {
void user_delay_ms(uint32_t ms);
int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr,uint8_t *reg_data, uint16_t len);
}


struct Measurement {
	float temp_C = 0;							// degrees Celsius
	float pressure_hPa = 0;				// hectoPascals
	float relHumidity = 0;				// % relative humidity
	std::optional<int> gas_ohms;	// raw gas measurement in sensor ohms
};

std::ostream& operator<<(std::ostream& os, const Measurement& m);


/// SPIDEV ////////////////////////////////////////////////////////////////////
class spiDev {
using Kptr = uint64_t;	// Linux kernel uses __u64 for all pointers
public:
	spiDev(fs::path device);
	~spiDev();

	int read(uint8_t reg, uint8_t* data, size_t bytes, uint16_t delay_us = 0);
	int write(uint8_t reg, uint8_t* data, size_t bytes);

private:
	int fd_;
};


/// BME680 ////////////////////////////////////////////////////////////////////
class bme680 {
public:
	bme680(fs::path device);
	~bme680();

	void configure();
	Measurement measure();

	static spiDev& lookup(uint8_t dev_id);

private:
	spiDev dev_;
	bme680_dev sensor_;
	static std::map<unsigned int, spiDev*> devMapper_;
};

#endif // LIBSPI_BME680_H

