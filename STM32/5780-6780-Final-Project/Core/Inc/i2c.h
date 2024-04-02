void initI2C(void);
void PrepareI2C2Transaction(uint32_t address, char RD_WRN, int numbytes);
void TransmissionWriteHelper(uint32_t address, int numbytes, uint32_t data);
char TransmissionReadHelper(uint32_t address, int numbytes);