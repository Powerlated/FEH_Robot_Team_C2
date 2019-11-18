#ifndef FEHFILE_H
#define FEHFILE_H

class FEHFile
{
	public:
		FIL wrapper;
		static int fileIdNum;
		FEHFile() {
			fileIdNum++;
		}
}; 
#endif
