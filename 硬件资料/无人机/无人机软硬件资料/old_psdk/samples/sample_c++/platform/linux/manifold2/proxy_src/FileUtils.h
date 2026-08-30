#pragma once
#include <string>
#include <vector>
#include <stdint.h>

class File 
{
public:
	std::string name;
	std::string path;
	int type;

public:
	static const int TYPE_FLIE = 1;
	static const int TYPE_DIR = 2;
};

class FileUtils
{
private:
	FileUtils(){};
	virtual ~FileUtils(){}
public:
	static int GetFileLength(const std::string& path);
	static int ReadFileData(const std::string& path, void* buffer, int offset, int size);
	static int WriteFileData(const std::string& path, void* buffer, int size);
	static int ListFiles(const std::string& dirpath, std::vector<File>& list, bool hide = true);
	static int MakeDirs(const std::string& dirpath);
	static int CreateFile(const std::string& path);
	static std::string Basename(const std::string& path);
	static std::string Dirname(const std::string& path);
public:
	static const char* SEPARATOR;
	static const char  SEPARATOR_CHAR;
};
