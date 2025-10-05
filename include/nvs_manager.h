#ifndef NVS_MANAGER_H
#define NVS_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sharedData.h"

typedef enum {
    TYPE_U8,
    TYPE_I8,
    TYPE_U16,
    TYPE_I16,
    TYPE_U32,
    TYPE_I32
} ValueType;

// Methods
void store_values(char *nvs_namespace, char *handle, ValueType _type, const void* val_ptr);

esp_err_t read_nvs_value(const char* namespaces[NS_COUNT], 
    const char* keys[NS_COUNT][MAX_KEYS_PER_NS], 
    const int key_counts[], 
    int ns_count,
    const char *ns_to_read,
    const char *key_to_read,
    ValueType _type,
    void *out_val);
    
void read_all_nvs_values(const char* namespaces[NS_COUNT],
     const char* keys[NS_COUNT][MAX_KEYS_PER_NS],
      const int key_counts[],
       int ns_count);

#ifdef __cplusplus
}
#endif

#endif // NVS_MANAGER_H