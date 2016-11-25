#define F_PATH "./param596.params"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mavlink.h>

struct paras{
	//int mav_id;
	//int component_id;
	char param_name[30];
	char value[30];
	//int value_num;
};

/*get value from every line*/
void get_params_from_line(char *_str, struct paras* __one_param){
	char *i, *j, *k; 
	i = _str+4;/*去掉了mav_id还有component_id*/
	j = __one_param->param_name;
	k = __one_param->value;
	while(!isspace(*i)){
		*(j++) = *i++;
	}
	*j = '\0'; /*每一个字符串都需要'\0'来结尾*/
	i++; /*跳过参数名和值之间的空格*/
	while(!isspace(*i)){
		*(k++) = *i++;
	}
	*k = '\0';
	//printf("%s %s\n", _param_one->param_name, _param_one->value);
	return;
}

/*read every line from params file*/
void get_line_from_file(struct paras _one_param){
	char str[60] = {};
	FILE *fp;
	if((NULL == (fp=fopen(F_PATH, "r")))){
		printf("不能打开此文件!\n");
		exit(0);
	}
	int n = 3;//去掉前三行注释
	while(!feof(fp)){
		fgets(str, 60, fp);
		n--;
		if(n<0){
			get_params_from_line(str, &_one_param);
			printf("%s %s\n", &_one_param.param_name, &_one_param.value);
		}
	}

	return;
}
/*
*MAV_CMD_DO_SET_PARAMETER
*/
int main(void){
	struct paras one_param;
	get_line_from_file(one_param);	
	mavlink_heartbeat_t my_heartbeat;
	return 0;
}
