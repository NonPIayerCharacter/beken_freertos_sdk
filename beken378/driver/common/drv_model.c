// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include.h"
#include "arm_arch.h"

#include "drv_model_pub.h"
#include "drv_model.h"
#include "mem_pub.h"
#include "str_pub.h"

static UINT32 allocated_size_sdev = 0;
static UINT32 allocated_size_dev  = 0;

static DRV_SDEV_PTR drv_sdev_tbl = NULLPTR;
static DRV_DEV_PTR  drv_dev_tbl  = NULLPTR;

UINT32 drv_model_init(void)
{
    drv_sdev_tbl = os_malloc(DD_INITIAL_SIZE * sizeof(DRV_SDEV_S));
    drv_dev_tbl  = os_malloc(DD_INITIAL_SIZE * sizeof(DRV_DEV_S ));

    ASSERT(drv_sdev_tbl);
    ASSERT(drv_dev_tbl );

    os_memset(drv_sdev_tbl, 0, DD_INITIAL_SIZE * sizeof(DRV_SDEV_S));
    os_memset(drv_dev_tbl, 0, DD_INITIAL_SIZE * sizeof(DRV_DEV_S ));

    allocated_size_sdev = DD_INITIAL_SIZE;
    allocated_size_dev  = DD_INITIAL_SIZE;

    return DRV_SUCCESS;
}

UINT32 drv_model_uninit(void)
{
    os_free(drv_sdev_tbl);
    os_free(drv_dev_tbl );

    drv_sdev_tbl = NULLPTR;
    drv_dev_tbl  = NULLPTR;

    allocated_size_sdev = 0;
    allocated_size_dev  = 0;

    return DRV_SUCCESS;
}

UINT32 ddev_check_handle(DD_HANDLE handle)
{
    UINT32 magic;
    UINT32 id;

    magic = handle & DD_HANDLE_MAGIC_MASK;
    id    = handle & DD_HANDLE_ID_MASK;
    if((DD_HANDLE_MAGIC_WORD == magic)
            && (id < allocated_size_dev))
    {
        return DRV_SUCCESS;
    }
    else
    {
        return DRV_FAILURE;
    }
}

DD_HANDLE ddev_make_handle(UINT32 id)
{
    UINT32 handle = DD_HANDLE_UNVALID;

    if(id >= allocated_size_dev)
    {
        goto make_exit;
    }

    handle = id + DD_HANDLE_MAGIC_WORD;

make_exit:
    return handle;
}

UINT32 ddev_get_id_from_handle(DD_HANDLE handle)
{
    UINT32 magic;
    UINT32 id;

    magic = handle & DD_HANDLE_MAGIC_MASK;
    id    = handle & DD_HANDLE_ID_MASK;

    if(magic != DD_HANDLE_MAGIC_WORD)
    {
        return DD_ID_UNVALID;
    }

    return id;
}

DD_HANDLE ddev_open(char *dev_name, UINT32 *status, UINT32 op_flag)
{
    UINT32 i;
    UINT32 handle;
    UINT32 name_len;
    DRV_DEV_PTR dev_ptr;
    DD_OPERATIONS *operation;
    GLOBAL_INT_DECLARATION();

    handle = DD_HANDLE_UNVALID;
    name_len = os_strlen(dev_name);
    if((!(dev_name && status)) || (name_len > DD_MAX_NAME_LEN))
    {
        goto open_exit;
    }

    *status = DRV_FAILURE;

    for(i = 0; i < allocated_size_dev; i ++)
    {
        dev_ptr = &drv_dev_tbl[i];
        if((dev_ptr)
                && (0 == os_strncmp(dev_ptr->name, dev_name, name_len)))
        {
            if(DD_STATE_OPENED == dev_ptr->state)
            {
                handle = ddev_make_handle(i);
            }
            else if(DD_STATE_CLOSED == dev_ptr->state)
            {
                handle = ddev_make_handle(i);

                operation = dev_ptr->op;
                if(operation && (operation->open))
                {
                    *status = (operation->open)(op_flag);
                }

                GLOBAL_INT_DISABLE();
                dev_ptr->state = DD_STATE_OPENED;
                dev_ptr->use_cnt = 0;
                GLOBAL_INT_RESTORE();
            }
            else
            {
            }

            GLOBAL_INT_DISABLE();
            dev_ptr->use_cnt ++;
            GLOBAL_INT_RESTORE();
            break;
        }
    }

    ASSERT(DD_HANDLE_UNVALID != handle);

open_exit:
    return handle;
}

UINT32 ddev_close(DD_HANDLE handle)
{
    UINT32 id;
    DRV_DEV_PTR dev_ptr;
    DD_OPERATIONS *operation;
    GLOBAL_INT_DECLARATION();

    id = ddev_get_id_from_handle(handle);
    if(DD_ID_UNVALID == id)
    {
        return DRV_FAILURE;
    }

    dev_ptr = &drv_dev_tbl[id];

    GLOBAL_INT_DISABLE();
    dev_ptr->use_cnt --;
    GLOBAL_INT_RESTORE();

    if(0 == dev_ptr->use_cnt)
    {
        operation = dev_ptr->op;
        if(operation && (operation->close))
        {
            (operation->close)();
        }

        ASSERT(dev_ptr);

        GLOBAL_INT_DISABLE();
        dev_ptr->state = DD_STATE_CLOSED;
        GLOBAL_INT_RESTORE();
    }

    return DRV_SUCCESS;
}

UINT32 ddev_read(DD_HANDLE handle, char *user_buf, UINT32 count, UINT32 op_flag)
{
    UINT32 id;
    UINT32 status;
    DRV_DEV_PTR dev_ptr;
    DD_OPERATIONS *operation;

    id = ddev_get_id_from_handle(handle);
    if(DD_ID_UNVALID == id)
    {
        return DRV_FAILURE;
    }

    status = DRV_FAILURE;
    dev_ptr = &drv_dev_tbl[id];
    ASSERT(dev_ptr);
    operation = dev_ptr->op;
    if(operation && (operation->read))
    {
        status = (operation->read)(user_buf, count, op_flag);
    }

    return status;
}

UINT32 ddev_write(DD_HANDLE handle, char *user_buf, UINT32 count, UINT32 op_flag)
{
    UINT32 id;
    UINT32 status;
    DRV_DEV_PTR dev_ptr;
    DD_OPERATIONS *operation;

    id = ddev_get_id_from_handle(handle);
    if(DD_ID_UNVALID == id)
    {
        return DRV_FAILURE;
    }

    status = DRV_FAILURE;
    dev_ptr = &drv_dev_tbl[id];
    ASSERT(dev_ptr);
    operation = dev_ptr->op;
    if(operation && (operation->write))
    {
        status = (operation->write)(user_buf, count, op_flag);
    }

    return status;
}

UINT32 ddev_control(DD_HANDLE handle, UINT32 cmd, VOID *param)
{
    UINT32 id;
    UINT32 status;
    DRV_DEV_PTR dev_ptr;
    DD_OPERATIONS *operation;

    id = ddev_get_id_from_handle(handle);
    if(DD_ID_UNVALID == id)
    {
        return DRV_FAILURE;
    }

    status = DRV_FAILURE;
    dev_ptr = &drv_dev_tbl[id];
    ASSERT(dev_ptr);
    operation = dev_ptr->op;
    if(operation && (operation->control))
    {
        status = (operation->control)(cmd, param);
    }

    return status;
}

UINT32 sddev_control(char *dev_name, UINT32 cmd, VOID *param)
{
    UINT32 i;
    UINT32 status;
    UINT32 name_len;
    DRV_SDEV_PTR dev_ptr;
    SDD_OPERATIONS *operation = NULLPTR;

    ASSERT(dev_name);
    status = DRV_FAILURE;
    name_len = os_strlen(dev_name);
    for(i = 0; i < allocated_size_sdev; i ++)
    {
        dev_ptr = &drv_sdev_tbl[i];
        if((dev_ptr)
                && (0 == os_strncmp(dev_ptr->name, dev_name, name_len)))
        {
            operation = dev_ptr->op;
            if(operation && (operation->control))
            {
                status = (operation->control)(cmd, param);
            }

            break;
        }
    }

    ASSERT(operation);

    return status;
}

UINT32 ddev_register_dev(char *dev_name, DD_OPERATIONS *optr)
{
    UINT32 i;
    DRV_DEV_PTR dev_ptr;

    if(!(dev_name && optr))
    {
        return DRV_FAILURE;
    }

    dev_ptr = NULLPTR;
    for(i = 0; i < allocated_size_dev; i ++)
    {
        dev_ptr = &drv_dev_tbl[i];
        if( (NULLPTR == dev_ptr->name)
                && (DD_STATE_NODEVICE == dev_ptr->state))
        {
            dev_ptr->name  = dev_name;
            dev_ptr->op    = optr;
            dev_ptr->state = DD_STATE_CLOSED;

            break;
        }
    }

    ASSERT(NULLPTR != dev_ptr->op);

    if (i == allocated_size_dev - 1)
    {
        dev_ptr = os_realloc(drv_dev_tbl, (DD_EXP_STEP_SIZE + allocated_size_dev) * sizeof(DRV_DEV_S));
        ASSERT(dev_ptr);

        /*newly allocated memory must be init*/
        os_memset(&dev_ptr[allocated_size_dev], 0, DD_EXP_STEP_SIZE * sizeof(DRV_DEV_S));
        allocated_size_dev += DD_EXP_STEP_SIZE;

        drv_dev_tbl = dev_ptr;
    }

    return DRV_SUCCESS;
}

UINT32 sddev_register_dev(char *dev_name, SDD_OPERATIONS *optr)
{
    UINT32 i;
    DRV_SDEV_PTR dev_ptr;

    if(!(dev_name && optr))
    {
        return DRV_FAILURE;
    }

    dev_ptr = NULLPTR;
    for(i = 0; i < allocated_size_sdev; i ++)
    {
        dev_ptr = &drv_sdev_tbl[i];
        if( (NULLPTR == dev_ptr->name)
                && (DD_STATE_NODEVICE == dev_ptr->state))
        {
            dev_ptr->name  = dev_name;
            dev_ptr->op    = optr;
            dev_ptr->state = DD_STATE_CLOSED;

            break;
        }
    }

    ASSERT(NULLPTR != dev_ptr->op);

    if (i == allocated_size_sdev - 1)
    {
        dev_ptr = os_realloc(drv_sdev_tbl, (DD_EXP_STEP_SIZE + allocated_size_sdev) * sizeof(DRV_SDEV_S));
        ASSERT(dev_ptr);

        /*newly allocated memory must be init*/
        os_memset(&dev_ptr[allocated_size_sdev], 0, DD_EXP_STEP_SIZE * sizeof(DRV_SDEV_S));
        allocated_size_sdev += DD_EXP_STEP_SIZE;

        drv_sdev_tbl = dev_ptr;
    }

    return DRV_SUCCESS;
}

UINT32 ddev_unregister_dev(char *dev_name)
{
    UINT32 i;
    UINT32 name_len;
    DRV_DEV_PTR dev_ptr;

    if(!dev_name)
    {
        return DRV_FAILURE;
    }

    dev_ptr = NULLPTR;
    name_len = os_strlen(dev_name);
    if((!dev_name) || (name_len > DD_MAX_NAME_LEN))
    {
        return DRV_FAILURE;
    }

    for(i = 0; i < allocated_size_dev; i ++)
    {
        dev_ptr = &drv_dev_tbl[i];
        if(0 == os_strncmp(dev_ptr->name, dev_name, name_len))
        {
            dev_ptr->name  = 0;
            dev_ptr->op    = 0;
            dev_ptr->state = DD_STATE_NODEVICE;

            dev_ptr        = NULLPTR;

            break;
        }
    }

    ASSERT(NULLPTR == dev_ptr);

    return DRV_SUCCESS;
}

UINT32 sddev_unregister_dev(char *dev_name)
{
    UINT32 i;
    UINT32 name_len;
    DRV_SDEV_PTR dev_ptr;

    if(!dev_name)
    {
        return DRV_FAILURE;
    }

    dev_ptr = NULLPTR;
    name_len = os_strlen(dev_name);
    if((!dev_name) || (name_len > DD_MAX_NAME_LEN))
    {
        return DRV_FAILURE;
    }

    for(i = 0; i < allocated_size_dev; i ++)
    {
        dev_ptr = &drv_sdev_tbl[i];
        if(0 == os_strncmp(dev_ptr->name, dev_name, name_len))
        {
            dev_ptr->name  = 0;
            dev_ptr->op    = 0;
            dev_ptr->state = DD_STATE_NODEVICE;

            dev_ptr        = NULLPTR;

            break;
        }
    }

    ASSERT(NULLPTR == dev_ptr);

    return DRV_SUCCESS;
}

// EOF
