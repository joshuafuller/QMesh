#ifndef FS_HPP
#define FS_HPP

// https://github.com/espressif/esp-idf/blob/5624dffc52271389376352974ba5911963ee207b/examples/storage/fatfsgen/main/fatfsgen_example_main.c

#if defined(MBED_OS)
#include "mbed.h"
#include "LittleFileSystem.h"
#include "QSPIFBlockDevice.h"
namespace portability {
    using FileSystem = ::LittleFileSystem;
    using BlockDevice = ::QSPIFBlockDevice;
} // namespace portability


#elif defined (ESP_IDF)
namespace portability {

class BlockDevice {
private:

public:
    void init() { }

    BlockDevice(PinName QSPI_IO0, 
                PinName QSPI_IO1, 
                PinName QSPI_IO2, 
                PinName QSPI_IO3, 
                PinName QSPI_SCK, 
                PinName QSPI_CSN, 
                int clock_mode,
                int freq) { }
};


class FileSystem {
private:
    string mount_point;
    // Handle of the wear levelling library instance
    wl_handle_t s_wl_handle{WL_INVALID_HANDLE};
    esp_vfs_fat_mount_config_t mount_config;

    auto prepend_mount_point(const char *path) -> string {
        string full_path = mount_point;
        full_path.append(path);
        return full_path;
    }

public:
    FileSystem() = delete;

    FileSystem(const char *mount_point) {
        s_wl_handle = WL_INVALID_HANDLE;
        mount_config.max_files = 4;
        mount_config.format_if_mount_failed = true;
        mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
        esp_err_t err = esp_vfs_fat_spiflash_mount(mount_point, "storage", &mount_config, &s_wl_handle);
        PORTABLE_ASSERT(err != ESP_OK);
    }

    auto remove(const char *path) -> int err_val {
        // implement stuff
        string full_path = prepend_mount_point(path);
        return remove(full_path);
    }

    auto rename(const char *old_path, const char *new_path) -> int err_val {
        string full_path_old = prepend_mount_point(old_path);
        string full_path_new = prepend_mount_point(new_path);
        return rename(full_path_old, full_path_new);
    }

    auto reformat(BlockDevice *bd) -> int err_val {
        // implement stuff

    }

    auto stat(const char *path, struct stat *st) -> int err_val {
        string full_path = prepend_mount_point(path);
        return stat(full_path.c_str(), st);
    }
};

} // namespace portability


#endif



#endif /* FS_HPP */