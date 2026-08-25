#include "GCodeParser.h"

#ifdef ARDUINO
File file;
#endif

int totalLines = 0;
int myconut = 0;

String readNextLine() {
#ifdef ARDUINO
  if (file && file.available())
    return file.readStringUntil('\n');
#endif
  return "";
}

bool parseCartesianLine(String line, float &x, float &y, float &z, float &e) {
  line.trim();
  if (line.length() == 0)
    return false;

  line.replace(",", " ");
  line.replace(";", " ");

  float val1, val2, val3, val4, val5;
  int parsedCount =
      sscanf(line.c_str(), "%f %f %f %f %f", &val1, &val2, &val3, &val4, &val5);

  if (parsedCount == 5) {
    x = val1;
    y = val2;
    z = val3;
    e = val5;
    return true;
  } else if (parsedCount == 4) {
    x = val1;
    y = val2;
    z = val3;
    e = val4;
    return true;
  }
  return false;
}
