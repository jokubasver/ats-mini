#ifndef CW_H
#define CW_H

#define CW_TEXT_LEN   64   // Maximum decoded text buffer length

void cwInit();
bool cwTickTime();
const char *getCwText();
void clearCwText();

#endif // CW_H
