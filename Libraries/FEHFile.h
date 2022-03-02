#ifndef FEHFILE_H
#define FEHFILE_H

/**
 * @brief Access to a file on the Proteus SD card
 * 
 * An instance of this class is returned by `FEHSD::FOpen`.
 */
class FEHFile
{
	public:
		// TODO make wrapper, fileIdNum, and constructor protected!
		FIL wrapper;
		static int prevFileId;
		int fileIdNum;
		FEHFile() {
			fileIdNum = ++prevFileId;
		}
}; 
#endif
