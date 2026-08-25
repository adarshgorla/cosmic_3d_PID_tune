#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include "Config.h"

#ifdef ARDUINO
#include "FS.h"
#include "SD.h"
extern File file;
#endif

extern int totalLines;
extern int myconut;

// SD File utilities & G-code line reader
String readNextLine();

// G-code / Cartesian target parsing
bool parseCartesianLine(String line, float &x, float &y, float &z, float &e);

#endif // GCODE_PARSER_H
