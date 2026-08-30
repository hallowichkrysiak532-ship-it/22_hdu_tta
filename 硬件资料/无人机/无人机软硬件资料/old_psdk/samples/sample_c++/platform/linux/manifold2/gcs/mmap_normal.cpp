#include "mmap_normal.h"

/*#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"


mmap_normal::mmap_normal()
{
    m_mmap_buf = NULL;
    m_filename = "/opt/ttaviation/global_share_memory.json";

    key1 = "cmd";
    key2 = "description";
    key3 = "response_index";
}

mmap_normal::~mmap_normal()
{
    if(m_mmap_buf != NULL)
    {
        munmap(m_mmap_buf, 4096);
    }
	
}

void mmap_normal::mmap_init()
{
    int fd;

    fd = open(m_filename.data(), O_RDWR, 00777);
    lseek(fd, 0, SEEK_SET);

    m_mmap_buf = (char*)mmap(NULL, 4096, PROT_READ|PROT_WRITE,MAP_SHARED, fd, 0);
    
    char* m = strchr(m_mmap_buf, '\r');
    *(m+1) = 0x00; //parse by str

    if (m_document.Parse(m_mmap_buf).HasParseError())
	{
        close(fd);
		return ;
	}

    close(fd);
}
 
void mmap_normal::mmap_write(const char* key, const char* value)
{
    cout << "mmap write!!"<< endl;
    //write mmap
    if (m_document.HasMember(key))
    {
        cout << "mmap key is------>>>>>>"<<key<< endl;

        m_document[key].SetString(value, strlen(value));

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        m_document.Accept(writer);
        std:string jsonstr =  buffer.GetString();
        jsonstr += "\n\r";  //in json

        memcpy(m_mmap_buf, jsonstr.data(), jsonstr.length());
    }
}

std::string mmap_normal::mmap_read(const char* key)
{
    std::string value;

    if (m_document.HasMember(key))
		value = m_document[key].IsString() ? m_document[key].GetString() : "";

    cout << "value is:"<< value << endl;

    return value;
}

*/
