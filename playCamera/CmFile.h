#pragma once
#include "stdafx.h"
#define CStr const string
#define vecS vector<string>
#define _S(a) a.c_str()

struct CmFile
{
	static string BrowseFile(const char* strFilter = "Images (*.jpg;*.png)\0*.jpg;*.png\0All (*.*)\0*.*\0\0", bool isOpen = true);
	static string BrowseFolder(); 


	static inline string GetFolder(CStr& path);
	static inline string GetSubFolder(CStr& path);
	static inline string GetName(CStr& path);
	static inline string GetNameNE(CStr& path);
	static inline string GetPathNE(CStr& path);
	static inline string GetNameNoSuffix(CStr& path, CStr &suffix);

	// Get file names from a wildcard. Eg: GetNames("D:\\*.jpg", imgNames);
	static int GetNames(CStr &nameW, vecS &names, string &dir);
	static int GetNames(CStr &nameW, vecS &names){string dir; return GetNames(nameW, names, dir);};
	static int GetNames(CStr& rootFolder, CStr &fileW, vecS &names);
	static int GetNamesNE(CStr& nameWC, vecS &names, string &dir, string &ext);
	static int GetNamesNE(CStr& rootFolder, CStr &fileW, vecS &names);
	static int GetNamesNE(CStr& nameWC, vecS &names) {string dir, ext; return GetNamesNE(nameWC, names, dir, ext);}
	static int GetNamesNoSuffix(CStr& nameWC, vecS &namesNS, CStr suffix, string &dir); //e.g. suffix = "_C.jpg"
	static int GetNamesNoSuffix(CStr& nameWC, vecS &namesNS, CStr suffix) {string dir; return GetNamesNoSuffix(nameWC, namesNS, suffix, dir);}

	static inline string GetExtention(CStr name);

	static inline bool FileExist(CStr& filePath);
	static inline bool FilesExist(CStr& fileW);
	static inline bool FolderExist(CStr& strPath);

	static inline string GetWkDir();

	static BOOL MkDir(CStr&  path);

	// Eg: RenameImages("D:/DogImages/*.jpg", "F:/Images", "dog", ".jpg");
	static int Rename(CStr& srcNames, CStr& dstDir, const char* nameCommon, const char* nameExt);
	static void RenameSuffix(CStr dir, CStr orgSuf, CStr dstSuf);

	static int ChangeImgFormat(CStr &imgW, CStr dstW); // "./*.jpg", "./Out/%s_O.png"




	static int GetSubFolders(CStr& folder, vecS& subFolders);
	static string GetFatherFolder(CStr &folder) {return GetFolder(folder.substr(0, folder.size() - 1));}

	inline static BOOL Copy(CStr &src, CStr &dst, BOOL failIfExist = FALSE);
	inline static BOOL Move(CStr &src, CStr &dst, DWORD dwFlags = MOVEFILE_REPLACE_EXISTING | MOVEFILE_COPY_ALLOWED | MOVEFILE_WRITE_THROUGH);
	static BOOL Move2Dir(CStr &srcW, CStr dstDir);
	static BOOL Copy2Dir(CStr &srcW, CStr dstDir);

	static void WriteNullFile(CStr& fileName) {FILE *f = fopen(_S(fileName), "w"); fclose(f);}
	static void AppendStr(CStr fileName, CStr str);



	static void RunProgram(CStr &fileName, CStr &parameters = "", bool waiteF = false, bool showW = true);
	static string GetCompName(); // Get the name of computer

	static void SegOmpThrdNum(double ratio = 0.8);

	// Copy files and add suffix. e.g. copyAddSuffix("./*.jpg", "./Imgs/", "_Img.jpg")
	static void copyAddSuffix(CStr &srcW, CStr &dstDir, CStr &dstSuffix);



};

/************************************************************************/
/* Implementation of inline functions                                   */
/************************************************************************/
string CmFile::GetFolder(CStr& path)
{
	return path.substr(0, path.find_last_of("\\/")+1);
}

string CmFile::GetSubFolder(CStr& path)
{
	string folder = path.substr(0, path.find_last_of("\\/"));
	return folder.substr(folder.find_last_of("\\/")+1);
}

string CmFile::GetName(CStr& path)
{
	int start = path.find_last_of("\\/")+1;
	int end = path.find_last_not_of(' ')+1;
	return path.substr(start, end - start);
}

string CmFile::GetNameNE(CStr& path)
{
	int start = path.find_last_of("\\/")+1;
	int end = path.find_last_of('.');
	if (end >= 0)
		return path.substr(start, end - start);
	else
		return path.substr(start,  path.find_last_not_of(' ')+1 - start);
}

string CmFile::GetNameNoSuffix(CStr& path, CStr &suffix)
{
	int start = path.find_last_of("\\/")+1;
	int end = path.size() - suffix.size();
	CV_Assert(path.substr(end) == suffix);
	if (end >= 0)
		return path.substr(start, end - start);
	else
		return path.substr(start,  path.find_last_not_of(' ')+1 - start);	
}

string CmFile::GetPathNE(CStr& path)
{
	int end = path.find_last_of('.');
	if (end >= 0)
		return path.substr(0, end);
	else
		return path.substr(0,  path.find_last_not_of(' ') + 1);
}

string CmFile::GetExtention(CStr name)
{
	return name.substr(name.find_last_of('.'));
}

BOOL CmFile::Copy(CStr &src, CStr &dst, BOOL failIfExist)
{
	return ::CopyFileA(src.c_str(), dst.c_str(), failIfExist);
}

BOOL CmFile::Move(CStr &src, CStr &dst, DWORD dwFlags)
{
	return MoveFileExA(src.c_str(), dst.c_str(), dwFlags);
}



// Test whether a file exist
bool CmFile::FileExist(CStr& filePath)
{
	if (filePath.size() == 0)
		return false;
	DWORD attr = GetFileAttributesA(_S(filePath));
	return attr == FILE_ATTRIBUTE_NORMAL ||  attr == FILE_ATTRIBUTE_ARCHIVE;//GetLastError() != ERROR_FILE_NOT_FOUND;
}

bool CmFile::FilesExist(CStr& fileW)
{
	vecS names;
	int fNum = GetNames(fileW, names);
	return fNum > 0;
}

string CmFile::GetWkDir()
{	
	string wd;
	wd.resize(1024);
	DWORD len = GetCurrentDirectoryA(1024, &wd[0]);
	wd.resize(len);
	return wd;
}

bool CmFile::FolderExist(CStr& strPath)
{
	int i = (int)strPath.size() - 1;
	for (; i >= 0 && (strPath[i] == '\\' || strPath[i] == '/'); i--)
		;
	string str = strPath.substr(0, i+1);

	WIN32_FIND_DATAA  wfd;
	HANDLE hFind = FindFirstFileA(_S(str), &wfd);
	bool rValue = (hFind != INVALID_HANDLE_VALUE) && (wfd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY);   
	FindClose(hFind);
	return rValue;
}

/************************************************************************/
/*                   Implementations                                    */
/************************************************************************/