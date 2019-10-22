#ifndef FEHFILE_H
#define FEHFILE_H

class FEHFile
{
public:
    FEHFile();
private:
	FIL* wrapper;
    int lineNumber;
};

#endif
