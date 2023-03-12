#include "datastorage.h"

//File gfdata(InternalFS);

//DataStorage::DataStorage(void) {
//    // file = new File(&InternalFS);
//}

DataStorage::~DataStorage(void) {
    if (fdata.isOpen())
        fdata.close();
}

void DataStorage::close(void) {
    if (fdata.isOpen())
        fdata.close();
}

bool DataStorage::is_file_open(void) {
    return fdata.isOpen();
}

bool DataStorage::open_file(File &file, const char *name, uint8_t mode) {
    return file.open(name, mode);
}

bool DataStorage::open_to_read(void) {
    this->close();
    if (this->open_file(fdata, FILEDATA, FILE_O_READ)) {
        current_f_perm = FILE_O_READ;
        return true;
    }
    return false;
}

bool DataStorage::open_to_write(void) {
    this->close();
    if (this->open_file(fdata, FILEDATA, FILE_O_WRITE)) {
        current_f_perm = FILE_O_WRITE;
        return true;
    }
    return false;
}

bool DataStorage::is_read_mode(void) {
    return current_f_perm == FILE_O_READ;
}

bool DataStorage::is_write_mode(void) {
    return current_f_perm == FILE_O_WRITE;
}

bool DataStorage::is_eof(void) {
    return fdata.available() <= 0;
}

uint32_t DataStorage::ftell(void) {
    return fdata.position();
}

uint32_t DataStorage::fsize(void) {
    return fdata.size();
}

bool DataStorage::fseek_set(uint32_t pos) {
    return fdata.seek(pos);
}

void DataStorage::restore_position(void) {
    if (this->open_file(fpos, FILEPOS, FILE_O_READ)) {
        uint32_t pos;
        uint32_t size = this->fsize();
        fpos.read((uint8_t *) &pos, sizeof(pos));
        fpos.close();
        Serial.print("Last position: ");
        Serial.print(pos);
        Serial.print(" Size: ");
        Serial.println(size);
        if (pos < size) {
            if (this->fseek_set(pos))
                Serial.println("Position restored!");
        }
    }
}

void DataStorage::save_position(void) {
    if (this->open_file(fpos, FILEPOS, FILE_O_WRITE)) {
        fpos.seek(0);
        uint32_t pos = this->ftell();
        fpos.write((uint8_t *) &pos, sizeof(pos));
        fpos.close();
        Serial.print("Saved position with value: ");
        Serial.println(pos);
    }
}

bool DataStorage::removeFile(void) {
    bool ret = false;
    if (InternalFS.exists(FILEDATA)) {
        this->close();
        ret = InternalFS.remove(FILEDATA) ? true : false;
        ret = ret && this->open_to_write() == true;
    }
    return ret;
}

void DataStorage::setup(void) {
    InternalFS.begin();
    /*
    if ( this->open_to_write() )
        Serial.println(FILEDATA " opened to write!");
    */
}

size_t DataStorage::read(uint8_t *buf, size_t size) {
    if (!fdata.isOpen())
        return 0;
    uint32_t readlen = 0;
    readlen = fdata.read(buf, size);
    uint32_t i;
    Serial.print("Read bytes: "); Serial.println(readlen);
    for (i = 0; i < readlen; i++) {
        Serial.print(buf[i], HEX);
    }
    Serial.println();
    return readlen;
}

size_t DataStorage::write(uint8_t const *buf, size_t size) {
    /*
    if (!fdata.isOpen())
        return 0;
    */
    if ( !this->open_to_write() )
        return 0;
    size_t writelen = fdata.write(buf, size);
    Serial.print("Write bytes: "); Serial.println(writelen);
    this->close();
    // if (writelen > 0)
    //     fdata.flush();
    return writelen;
}

