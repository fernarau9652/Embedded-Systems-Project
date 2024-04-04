void initI2C(void);
void PrepareI2C2Transaction(__UINT32_TYPE__ address, char RD_WRN, int numbytes);
void TransmissionWriteHelper(__UINT32_TYPE__ address, int numbytes, __UINT32_TYPE__ data);
char TransmissionReadHelper(__UINT32_TYPE__ address, int numbytes);