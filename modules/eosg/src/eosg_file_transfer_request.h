#pragma once
#include "core/object/gdvirtual.gen.inc"
#include "core/variant/dictionary.h"
#include "core/object/ref_counted.h"


class EOSGFileTransferRequest : public RefCounted {
    GDCLASS(EOSGFileTransferRequest, RefCounted)

    GDVIRTUAL0R(int, get_file_request_state)
    GDVIRTUAL0R(Dictionary, get_filename)
    GDVIRTUAL0R(int, cancel_request)

private:
    static void _bind_methods() {
        GDVIRTUAL_BIND(get_file_request_state);
        GDVIRTUAL_BIND(get_filename);
        GDVIRTUAL_BIND(cancel_request);
    };

public:
    virtual int get_file_request_state() {
        return -1;
    }
    virtual Dictionary get_filename() {
        return Dictionary();
    }
    virtual int cancel_request() {
        return -1;
    }

    EOSGFileTransferRequest(){};
    ~EOSGFileTransferRequest(){};
};
