#include "FileUtils.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <libgen.h>

#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
const char* FileUtils::SEPARATOR = "\\";
const char  FileUtils::SEPARATOR_CHAR = '\\';
#else
const char* FileUtils::SEPARATOR = "/";
const char  FileUtils::SEPARATOR_CHAR = '/';
#endif

#if defined(__WINDOWS__) || defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64) || \
	defined(__WIN32__) || defined(__TOS_WIN__)
#define MKDIR(x) _mkdir(x)
#else
#define MKDIR(x) mkdir(x, S_IRWXU | S_IRWXG | S_IRWXO)
#endif

int FileUtils::GetFileLength(const std::string& path)
{
	struct stat buf;
	int err = stat(path.c_str(), &buf);
	if(err==0) {
		if(buf.st_mode & S_IFREG) {
			return buf.st_size;
		}else{
			return -1;
		}
	}else {
		return -1;
	}
}
int FileUtils::ReadFileData(const std::string& path, void* buffer, int offset, int size)
{
	FILE* fp = fopen(path.c_str(), "rb");
	if(fp) {
		int r = 0;
		fseek(fp, offset, SEEK_SET);
		r = fread(buffer, 1, size, fp);
		fclose(fp);
		return r;
	}else{
		return -1;
	}
}
int FileUtils::WriteFileData(const std::string& path, void* buffer, int size)
{
	FILE* fp = fopen(path.c_str(), "a");
	if(fp) {
		int w = fwrite(buffer, 1, size, fp);
		fclose(fp);
		return w;
	}else{
		return -1;
	}
}
int FileUtils::ListFiles(const std::string& dirpath, std::vector<File>& list, bool hide)
{
	DIR* dir = NULL;
	struct dirent* ptr = NULL;
	if((dir=opendir(dirpath.c_str())) == NULL){
		return -1;
	}
	while ((ptr=readdir(dir)) != NULL)
	{
		if(strcmp(ptr->d_name, ".")==0 || strcmp(ptr->d_name, "..")==0){
			continue;
		}else if(ptr->d_type == 8){ //file
			if(!hide || ptr->d_name[0]!='.'){
				File file;
				file.name = ptr->d_name;
				file.path = dirpath + SEPARATOR + file.name;
				file.type = File::TYPE_FLIE;
				list.push_back(file);
			}
		}else if(ptr->d_type == 10){ //link file
			continue;
		}else if(ptr->d_type == 4){ //dir
			if(!hide || ptr->d_name[0]!='.'){
				File file;
				file.name = ptr->d_name;
				file.path = dirpath + SEPARATOR + file.name;
				file.type = File::TYPE_DIR;
				list.push_back(file);
			}
		}
	}
	closedir(dir);
	return 0;
}

int FileUtils::MakeDirs(const std::string& dirpath)
{
	if(dirpath.length()<=0 || dirpath.length()>PATH_MAX){
		return -1;
	}
	size_t t = -1;
	while((t=dirpath.find(SEPARATOR_CHAR, t+1))!=std::string::npos){
		std::string str = dirpath.substr(0, t);
		MKDIR(str.c_str());
	}
	MKDIR(dirpath.c_str());
	return 0;
}

int FileUtils::CreateFile(const std::string& path)
{
	std::string dir = Dirname(path);
	MakeDirs(dir);
	FILE* fp = fopen(path.c_str(), "w");
	if(fp) {
		fclose(fp);
		return 0;
	}else{
		return -1;
	}
}
std::string FileUtils::Basename(const std::string& path)
{
	char *basec, *bname;
    basec = strdup(path.c_str());
	bname = basename(basec);
	std::string ret = bname;
	free(basec);
	return ret;
}
std::string FileUtils::Dirname(const std::string& path)
{
	char *dirc, *dname;
    dirc = strdup(path.c_str());
	dname = dirname(dirc);
	std::string ret = dname;
	free(dirc);
	return ret;
}