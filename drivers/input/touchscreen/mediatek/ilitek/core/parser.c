/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include "../common.h"
#include "parser.h"

#define PARSER_MAX_CFG_BUF          512
#define PARSER_MAX_KEY_NUM	        600
#define PARSER_MAX_KEY_NAME_LEN	    100
#define PARSER_MAX_KEY_VALUE_LEN	2000

#define INI_ERR_OUT_OF_LINE     -1

struct ini_file_data
{
	char pSectionName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyValue[PARSER_MAX_KEY_VALUE_LEN];
	int iSectionNameLen;
	int iKeyNameLen;
	int iKeyValueLen;
} ilitek_ini_file_data[PARSER_MAX_KEY_NUM];

int _gINIItems = 0;
static int ilitek_inisize = 0;
static char *ini_str_trim_r(char * buf)
{
	int len, i;
	char tmp[512] = {0};

    len = strlen(buf);

    for(i = 0;i < len;i++)
    {
		if (buf[i] !=' ')
			break;
    }

    if (i < len)
		strncpy(tmp,(buf+i),(len-i));

	strncpy(buf,tmp,len);
	return buf;
}

/* Count the number of each line and assign the content to tmp buffer */
static int get_ini_phy_line(char *data, char *buffer, int maxlen)
{
	int i=0;
	int j=0;
	int iRetNum=-1;
	char ch1='\0';

	for(i=0, j=0; i<maxlen; j++) {
        ch1 = data[j];
		iRetNum = j+1;
		if(ch1 == '\n' || ch1 == '\r') //line end
		{
			ch1 = data[j+1];
			if(ch1 == '\n' || ch1 == '\r')
			{
				iRetNum++;
			}

			break;
		}else if(ch1 == 0x00)
		{
			iRetNum = -1;
			break; //file end
		}
		else
		{
			buffer[i++] = ch1;
		}
	}
	buffer[i] = '\0';

	return iRetNum;
}

static int get_ini_phy_data(char *data)
{
    int i, n = 0, res = 0;
    int offset = 0, isEqualSign = 0;
    char *ini_buf = NULL, *tmpSectionName = NULL;
    char M_CFG_SSL = '[';
    char M_CFG_SSR = ']';
//    char M_CFG_NIS = ':';
    char M_CFG_NTS = '#';
    char M_CFG_EQS = '=';

    if(data == NULL)
    {
        DBG_ERR("INI data is NULL\n");
        res = -EINVAL;
        goto out;
    }

    ini_buf = kzalloc((PARSER_MAX_CFG_BUF + 1) * sizeof(char), GFP_KERNEL);
    if(ERR_ALLOC_MEM(ini_buf))
    {
        DBG_ERR("Failed to allocate ini_buf memory, %ld\n", PTR_ERR(ini_buf));
        res = -ENOMEM;
        goto out;
    }

    tmpSectionName = kzalloc((PARSER_MAX_CFG_BUF + 1)  * sizeof(char), GFP_KERNEL);
    if(ERR_ALLOC_MEM(tmpSectionName))
    {
        DBG_ERR("Failed to allocate tmpSectionName memory, %ld\n", PTR_ERR(tmpSectionName));
        res = -ENOMEM;
        goto out;
    }

    while(true)
    {
        if(_gINIItems > PARSER_MAX_KEY_NUM)
        {
            DBG_ERR("MAX_KEY_NUM: Out of length \n");
            goto out;
        }
		if (offset >= ilitek_inisize)
		{
            DBG_ERR("(offset >= ilitek_inisize) offset = %d\n", offset);
		}
        n = get_ini_phy_line(data + offset, ini_buf, PARSER_MAX_CFG_BUF);

        if(n < 0)
        {
            DBG_ERR("End of Line\n");
            goto out;
        }

        offset += n;

        n = strlen(ini_str_trim_r(ini_buf));

		if(n == 0 || ini_buf[0] == M_CFG_NTS)
            continue;

        /* Get section names */
		if(n > 2 && ((ini_buf[0] == M_CFG_SSL && ini_buf[n-1] != M_CFG_SSR)))
		{
			DBG_ERR("Bad Section: %s \n", ini_buf);
            res = -EINVAL;
            goto out;
        }
        else
        {
            if(ini_buf[0] == M_CFG_SSL)
            {
                ilitek_ini_file_data[_gINIItems].iSectionNameLen = n-2;
                if(PARSER_MAX_KEY_NAME_LEN < ilitek_ini_file_data[_gINIItems].iSectionNameLen)
                {
                    DBG_ERR("MAX_KEY_NAME_LEN: Out Of Length\n");
                    res = INI_ERR_OUT_OF_LINE;
                    goto out;
                }

                ini_buf[n-1] = 0x00;
                strcpy((char *)tmpSectionName, ini_buf+1);
                DBG(DEBUG_PARSER,"Section Name: %s, Len: %d \n", tmpSectionName, n-2);
                continue;
            }
        }

        /* copy section's name without square brackets to its real buffer */
        strcpy(ilitek_ini_file_data[_gINIItems].pSectionName, tmpSectionName);
        ilitek_ini_file_data[_gINIItems].iSectionNameLen = strlen(tmpSectionName);


        isEqualSign = 0;
		for(i=0; i < n; i++)
		{
			if(ini_buf[i] == M_CFG_EQS )
			{
				isEqualSign = i;
				break;
			}
        }

		if(0 == isEqualSign)
            continue;

        /* Get Key names */
        ilitek_ini_file_data[_gINIItems].iKeyNameLen = isEqualSign;
		if(PARSER_MAX_KEY_NAME_LEN < ilitek_ini_file_data[_gINIItems].iKeyNameLen)
		{
				//ret = CFG_ERR_OUT_OF_LEN;
                DBG_ERR("MAX_KEY_NAME_LEN: Out Of Length\n");
                res = INI_ERR_OUT_OF_LINE;
                goto out;
		}

		memcpy(ilitek_ini_file_data[_gINIItems].pKeyName,
            ini_buf, ilitek_ini_file_data[_gINIItems].iKeyNameLen);

        /* Get a value assigned to a key */
		ilitek_ini_file_data[_gINIItems].iKeyValueLen = n-isEqualSign-1;
		if(PARSER_MAX_KEY_VALUE_LEN < ilitek_ini_file_data[_gINIItems].iKeyValueLen)
		{
                DBG_ERR("MAX_KEY_VALUE_LEN: Out Of Length\n");
                res = INI_ERR_OUT_OF_LINE;
				goto out;
		}

		memcpy(ilitek_ini_file_data[_gINIItems].pKeyValue,
            ini_buf+isEqualSign+1, ilitek_ini_file_data[_gINIItems].iKeyValueLen);

        DBG(DEBUG_PARSER,"%s = %s \n",ilitek_ini_file_data[_gINIItems].pKeyName, ilitek_ini_file_data[_gINIItems].pKeyValue);

        _gINIItems++;
    }

out:
    kfree(ini_buf);
    kfree(tmpSectionName);
    return res;
}

static void init_ilitek_ini_data(void)
{
    int i;

    _gINIItems = 0;

    /* Initialise ini strcture */
	for(i = 0; i < PARSER_MAX_KEY_NUM; i++)
	{
		memset(ilitek_ini_file_data[i].pSectionName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ilitek_ini_file_data[i].pKeyName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ilitek_ini_file_data[i].pKeyValue, 0, PARSER_MAX_KEY_VALUE_LEN);
		ilitek_ini_file_data[i].iSectionNameLen = 0;
		ilitek_ini_file_data[i].iKeyNameLen = 0;
		ilitek_ini_file_data[i].iKeyValueLen = 0;
	}
}

/* get_ini_key_value - get ini's key and value based on its section from its array
 *
 * A function is digging into the key and value by its section from the ini array.
 * The comparsion is not only a string's name, but its length.
 */
static int get_ini_key_value(char * section, char * key, char * value)
{
	int i = 0;
	int ret = -2;
	int len = 0;

	len = strlen(key);

	for(i = 0; i < _gINIItems; i++)
	{
			if(strcmp(section, ilitek_ini_file_data[i].pSectionName) != 0)
                 continue;

			if(strcmp(key, ilitek_ini_file_data[i].pKeyName) == 0)
			{
				memcpy(value, ilitek_ini_file_data[i].pKeyValue, ilitek_ini_file_data[i].iKeyValueLen);
                DBG(DEBUG_PARSER," value:%s , pKeyValue: %s \n",value, ilitek_ini_file_data[i].pKeyValue);
				ret = 0;
				break;
			}
	}
	return ret;
}

/* core_parser_get_ini_data - Get ini real value by its key & section
 *
 * An interface exporting to outside is used to get INI real value according to its key and section.
 *
 * @section: Section name
 * @keyname: Key name
 * @rv : A value as a string returning to callers depends on the key and the section.
 */
int core_parser_get_int_data(char *section, char *keyname, char *rv)
{
    int len = 0;
    char value[512] = {0};

    if(rv == NULL || section == NULL || keyname == NULL)
    {
        DBG_ERR("Parameters are invalid\n");
        return -EINVAL;
    }

    /* return a white-space string if get nothing */
    if(get_ini_key_value(section, keyname, value) < 0)
    {
        sprintf(rv, "%s", value);
        return 0;
    }
    else
    {
        len = sprintf(rv, "%s", value);
        return len;
    }
}
EXPORT_SYMBOL(core_parser_get_int_data);

int core_parser_path(char *path)
{
    int res = 0, fsize = 0;
    char *tmp = NULL;
    struct file *f = NULL;
    struct inode *inode;
    mm_segment_t old_fs;
    loff_t pos = 0;

    DBG_INFO("path = %s \n", path);
    f = filp_open(path, O_RDONLY, 0);
    if (ERR_ALLOC_MEM(f))
    {
        DBG_ERR("Failed to open the file at %ld.\n", PTR_ERR(f));
        res = -ENOENT;
        return res;
    }

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 18, 0)
    inode = f->f_dentry->d_inode;
#else
    inode = f->f_path.dentry->d_inode;
#endif

    fsize = inode->i_size;
    DBG_INFO("fsize = %d",fsize);
    if (fsize <= 0)
    {
        DBG_ERR("The size of file is invaild\n");
        res = -EINVAL;
        goto out;
    }
	ilitek_inisize = fsize;
    //tmp = kzalloc(fsize * sizeof(char), GFP_KERNEL);
    tmp = vmalloc(fsize * sizeof(char) + PARSER_MAX_CFG_BUF);
    if(ERR_ALLOC_MEM(tmp))
    {
        DBG_ERR("Failed to allocate tmp memory, %ld\n", PTR_ERR(tmp));
        res = -ENOMEM;
        goto out;
    }
	memset(tmp, 0x00, fsize * sizeof(char) + PARSER_MAX_CFG_BUF);
    /* ready to map user's memory to obtain data by reading files */
    old_fs = get_fs();
    set_fs(get_ds());
    vfs_read(f, tmp, fsize, &pos);
    set_fs(old_fs);

    init_ilitek_ini_data();

    res = get_ini_phy_data(tmp);
    if(res < 0)
    {
        DBG_ERR("Failed to get physical ini data, res = %d", res);
        goto out;
    }

    DBG_INFO("Parsing INI file doen \n");

out:
    //kfree(tmp);
    vfree(tmp);
	tmp = NULL;
    return res;
}
EXPORT_SYMBOL(core_parser_path);
