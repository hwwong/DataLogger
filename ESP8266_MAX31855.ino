



void setDataBits(uint16_t bits) {
  const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
  bits--;
  SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}


void max31855Setup() {
  //define pin modes
  pinMode(MAX31855_CS_PIN, OUTPUT);
  //  //start and configure hardware SPI
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(5000000);
  setDataBits(32);
  digitalWrite(MAX31855_CS_PIN, HIGH);
}


int16_t max31855_read() {
  digitalWrite(MAX31855_CS_PIN, LOW);

  while (SPI1CMD & SPIBUSY) {}
  SPI1CMD |= SPIBUSY;     // SPI send dummy
  while (SPI1CMD & SPIBUSY) {}

  digitalWrite(MAX31855_CS_PIN, HIGH);

  //check error bit
  if (SPI1W0 & 0x100) {
    return 0xffffffff;    // return 0xffffffff. if error
  } else {
    // check netvigative value
    if (((byte*)&SPI1W0)[0] & 0x80)
      return (((((byte*)&SPI1W0)[0] | 0xffC0) << 6) ) | ((byte*)&SPI1W0)[1] >> 2 ;
    else
      return  ((byte*)&SPI1W0)[0] << 6 | ((byte*)&SPI1W0)[1] >> 2;
  }

}


