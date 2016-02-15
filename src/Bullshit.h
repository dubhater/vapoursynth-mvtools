#ifndef BULLSHIT_H
#define BULLSHIT_H

#ifdef _MSC_VER
// Don't forget to zero the last byte. _snprintf doesn't do it if the string doesn't fit.
#define snprintf _snprintf
#endif

#endif // BULLSHIT_H
