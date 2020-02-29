# -*- coding: utf-8 -*- 
import os 
import math
import sys

tag_num = 80

all_result = {}
list_dirs = os.walk(sys.argv[1]) 
for root, dirs, files in list_dirs: 
    for file_name in files:
        if root.find("test_def") >= 0 and file_name.find("bitrate1000") >=0 and file_name.find("lane2") >= 0 and file_name.find("velocity1") >= 0 :
            key_list = file_name.split('_')
            k = key_list[0] + '_' + key_list[4] + '_'+key_list[5].split('.')[0]
#            spacing = int(key_list[5].split('.')[0][len("spacing"):])

#            print(file_name)
            f = open(os.path.join(root, file_name), "r")
            lines = f.readlines()
#            print(len(lines))
            for line in lines:
                all_result.setdefault(k, []).append([int(line.split(' ')[2]), int(line.split(' ')[9])])
            f.close()
write_res = {}
#print(all_result)
for k in all_result:
    res_single = 0
    res_total = 0
    total_tag_num = 0
    spacing = int(k.split("_")[2][len("spacing"):])
    col_num = int(k.split("_")[1][len("collocated"):])
#    total_dist = spacing * tag_num
#    insert_num = total_dist // 1000 * col_num
#    print("spacing:"+str(spacing)+"   total_dist:"+str(total_dist)+"  insert_num:"+str(insert_num))
    for val in all_result[k]:
        res_single += val[0]
        res_total += val[1]
#        if val > (insert_num+tag_num):
#            print(val, insert_num+tag_num)
#    res_single /= ((tag_num + insert_num) * len(all_result[k]))
#    print(res_single)
    res_single /= res_total
    res_single = 1 - res_single
    res_single *= 100
    #print(int(k.split("_")[2][len("spacing"):]))
    write_res.setdefault(k.split("_")[0]+ "_" + k.split("_")[1], []).append([int(k.split("_")[2][len("spacing"):]), res_single])

print(write_res)
for k in write_res:
    f_res = open("TAG_DENSITY/"+k+'.txt', "w+")
    write_res[k].sort()
    f_res.write(str(write_res[k]).replace(']]', '\n').replace('],', '\n').replace('[','').replace(',', ' '))
    f_res.close()

