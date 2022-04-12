#include <Arduino.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#define FILEDATA    "/sensordata.txt"
#define FILEPOS     "/pos_sensordata"
#define CONTENTS    "Adafruit Little File System test file contents"

using namespace Adafruit_LittleFS_Namespace;

//extern File gfile;

class DataStorage {
    public:
        /* Constructor */
  //      DataStorage(void);
        ~DataStorage(void);

        void   setup(void);
        void   close(void);
        bool   open_file(File &file, const char *name,
                         uint8_t mode);
        bool   open_to_read(void);
        bool   open_to_write(void);
        bool   is_file_open(void);
        bool   is_eof(void);
        bool   is_read_mode(void);
        bool   is_write_mode(void);
        uint32_t ftell(void);
        uint32_t fsize(void);
        bool   fseek_set(uint32_t pos);
        void   restore_position(void);
        void   save_position(void);
        bool   removeFile(void);
        size_t read(uint8_t *buf, size_t size);
        size_t write(uint8_t chr);
        size_t write(uint8_t const *buf, size_t size);
    private:
        File fdata = File(InternalFS);
        File fpos  = File(InternalFS);
        int8_t current_f_perm = -1;
};
