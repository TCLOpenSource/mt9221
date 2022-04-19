#include <linux/uaccess.h>
#define LIST_MAX_LENGTH 128
bool found_Special_Task(char* searchSource, char* pattern,int len);
extern char TaskStatus_name_list[LIST_MAX_LENGTH];
int Get_Target_pid(void);
int Set_Target_pid(int pid);
int Delete_Target_pid(void);
int Get_Target_pid_OOM_adj(void);
int Set_Target_pid_OOM_adj(int oom_adj);
