/* This is a wrapper layer for optee 2.4 and optee 1.0.1 api compatibility
 * Can pass any types arguments through using void * and can pass arbitrary
 * number of arguments that must less than api declaration.
 */

#ifdef CONFIG_TEE_2_4
#include <linux/types.h>
#include <linux/device.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,19,0)

/* tee_shm_pool_free */
extern int tee_shm_pool_free_1_0_1(void *, void *, void *, void *);
extern void tee_shm_pool_free_2_4(void *);

/* tee_shm_alloc */
extern void *tee_shm_alloc_1_0_1(void *, void *, void *);
extern void *tee_shm_alloc_2_4(void *, void *, void *);
extern void *tee_shm_alloc_3_2(void *, void *, void *);

/* tee_shm_free */
extern void tee_shm_free_1_0_1(void *);
extern void tee_shm_free_2_4(void *);

/* tee_shm_put */
extern void tee_shm_put_1_0_1(void *, void *);
extern void tee_shm_put_2_4(void *);

/* tee_shm_pool_alloc */
extern void *tee_shm_pool_alloc_1_0_1(void *, void *, void *, void *);
extern void *tee_shm_pool_alloc_3_2(void *, void *);

static int get_optee_version(void)
{
        extern int optee_version;
        return optee_version;
}
int tee_shm_pool_free(void *a0, void *a1, void *a2, void *a3)
{
    int tee_ver = get_optee_version();
    switch(tee_ver) {
        case 1: //optee 1.0.1
        {
            return tee_shm_pool_free_1_0_1(a0, a1, a2, a3);
        }
        case 2: //optee 2.4
        {
            tee_shm_pool_free_2_4(a0);
            return 1;
        }
        case 3: //optee 3.2 also use 2.4 api
        {
            tee_shm_pool_free_2_4(a0);
            return 1;
        }
        default:
        {
            printk("%s(%d) Invalid optee version: %d \n", __FUNCTION__, __LINE__, tee_ver);
            break;
        }
    }
    return 0;
}
EXPORT_SYMBOL_GPL(tee_shm_pool_free);

void *tee_shm_alloc(void *a0, void *a1, void *a2)
{
    int tee_ver = get_optee_version();
    switch(tee_ver) {
        case 1: //optee 1.0.1
        {
            return tee_shm_alloc_1_0_1(a0, a1, a2);
        }
#ifndef CONFIG_TEE_3_2
        case 2: //optee 2.4
        {
            return tee_shm_alloc_2_4(a0, a1, a2);
        }
#else
        case 3: //optee 3.2
        {
            return tee_shm_alloc_3_2(a0, a1, a2);
        }
#endif
        default:
        {
            printk("%s(%d) Invalid optee version: %d \n", __FUNCTION__, __LINE__, tee_ver);
            break;
        }
    }
    return NULL;
}
EXPORT_SYMBOL_GPL(tee_shm_alloc);

void tee_shm_free(void *a0)
{
    int tee_ver = get_optee_version();
    switch(tee_ver) {
        case 1: //optee 1.0.1
        {
            tee_shm_free_1_0_1(a0);
            break;
        }
        case 2: //optee 2.4
        {
            tee_shm_free_2_4(a0);
            break;
        }
        case 3: //optee 3.2 also use 2.4 api
        {
            tee_shm_free_2_4(a0);
            break;
        }
        default:
        {
            printk("%s(%d) Invalid optee version: %d \n", __FUNCTION__, __LINE__, tee_ver);
            break;
        }
    }
}
EXPORT_SYMBOL_GPL(tee_shm_free);

void tee_shm_put(void *a0, void *a1)
{
    int tee_ver = get_optee_version();
    switch(tee_ver) {
        case 1: //optee 1.0.1
        {
            tee_shm_put_1_0_1(a0, a1);
            break;
        }
        case 2: //optee 2.4
        {
            tee_shm_put_2_4(a0);
            break;
        }
        case 3: //optee 3.2 also use 2.4 api
        {
            tee_shm_put_2_4(a0);
            break;
        }
        default:
        {
            printk("%s(%d) Invalid optee version: %d \n", __FUNCTION__, __LINE__, tee_ver);
            break;
        }
    }

}
EXPORT_SYMBOL_GPL(tee_shm_put);

#ifdef CONFIG_TEE_3_2
void *tee_shm_pool_alloc(void *a0, void *a1, void *a2, void *a3)
{
    int tee_ver = get_optee_version();
    switch(tee_ver) {
        case 1: //optee 1.0.1
        {
            return tee_shm_pool_alloc_1_0_1(a0, a1, a2, a3);
        }
        case 3: //optee 3.2
        {
            return tee_shm_pool_alloc_3_2(a0, a1);
        }
        default:
        {
            printk("%s(%d) Invalid optee version: %d \n", __FUNCTION__, __LINE__, tee_ver);
            break;
        }
    }
    return NULL;

}
EXPORT_SYMBOL_GPL(tee_shm_pool_alloc);
#endif
#endif
#endif
