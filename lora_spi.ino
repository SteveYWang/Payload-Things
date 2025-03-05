// SoftwareSPI.h
// Author: Chris Lapa (chris@lapa.com.au)
// Copyright (C) 2014 Chris Lapa
// Contributed by Chris Lapa

#include <RHGenericSPI.h>
#include <SPI.h>

class RHLoRaSPI : public RHGenericSPI 
{
private:
    SPIClass *_spi;

public:

    /// Constructor
    RHLoRaSPI(SPIClass *spi, int freq) : RHGenericSPI(Frequency1MHz, BitOrderMSBFirst, DataMode0), _spi(spi) { // note: many of these settings hardcoded in beginTransaction
      if (freq != 1000000) {
        log("WARNING: LoRa SPI initialized with non-1MHz frequency, must update GenericSPI constructor");
      }
    }

    /// Transfer a single octet to and from the SPI interface
    /// \param[in] data The octet to send
    /// \return The octet read from SPI while the data octet was sent.
    uint8_t transfer(uint8_t data) {
      return _spi->transfer(data);
    }

    /// Initialise the software SPI library
    /// Call this after configuring the SPI interface and before using it to transfer data.
    /// Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high. 
    void begin() {
      // TODO: don't think we need anything? already did this outside
    }

    /// Disables the SPI bus usually, in this case
    /// there is no hardware controller to disable.
    void end() {
      // TODO: don't really want anything here either, since it's shared?
    }

    // Forward transaction start/end to hardware SPI
    void beginTransaction() {
      spiLock(portMAX_DELAY);
      _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // NOTE: hardcoded settings since only use in one place, not a real library
    }
    void endTransaction() {
      _spi->endTransaction();
      spiUnlock();
    }

};

