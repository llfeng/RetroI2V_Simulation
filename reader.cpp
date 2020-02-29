/*
 * =====================================================================================
 *
 *       Filename:  reader.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2019年07月29日 15时00分50秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  lilei.feng , lilei.feng@pku.edu.cn
 *        Company:  Peking University
 *
 * =====================================================================================
 */
//TODO LIST:




#include <iostream>     // std::cout
#include <algorithm>    // std::sort
#include <vector>       // std::vector
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <sys/stat.h> 
#include <sys/time.h>
#include "crc8.h"
#include "is_available.h"
#include "lambertian.h"
#include "log.h"


uint64_t test_counter = 0;

using namespace std;

typedef struct{
    int uplink_bitrate;
    int uplink_max_distance;
    int lane_num;
    int velocity_type;
    int collocated_tag_num;
}case_t;

typedef struct{
    uint16_t device_index[2];
    uint64_t start_time;
    uint64_t end_time;
}collision_t;

typedef struct{
    uint16_t srcAddr;
    uint8_t dstAddr;
    uint8_t type;
    uint8_t ack_flag;
    uint8_t round;
    uint8_t ack_crc8;
    uint8_t collision_num;
    uint8_t overhear_count;
    vector<uint8_t> overhear_buf;
    vector<uint64_t> overhear_time;

    uint8_t buf[2048];
    uint16_t buflen;
}reader_output_t;

typedef struct{
    uint8_t type;
    uint16_t dstAddr;
    int8_t srcAddr;
    uint8_t plen;
    uint8_t round;
    uint8_t payload[128];

    uint8_t buf[256];
    uint16_t buflen;
}reader_input_t;

typedef struct{
    uint64_t send_start_time;
    uint64_t send_end_time;
    uint64_t check_time;
    uint8_t state;
    double init_posx;
    double init_posy;
    double velocity;
    double max_distance;
    double fov;
    uint32_t ID;
    uint16_t index;

    uint8_t collision_occur;
    uint8_t window;
    uint8_t need_backoff_flag;

    vector<uint8_t> tagID;

    vector<collision_t> collisions;
    reader_output_t output;
    reader_input_t input;
}reader_t;

typedef reader_output_t tag_input_t;
typedef reader_input_t tag_output_t;

typedef struct{
    uint16_t addr;   
}tag_quiet_t;

typedef struct{
    uint32_t addr;
    uint8_t sent;
    uint8_t ack_crc8;
}tag_alias_t;

typedef struct{
    uint64_t send_start_time;
    uint64_t send_end_time;
    uint64_t check_time;
    uint8_t state;
    double init_posx;
    double init_posy;
    double max_distance;
    int uplink_bitrate;
    double fov;
    uint32_t ID;
    uint16_t index;
    vector<collision_t> collisions;
    vector<tag_quiet_t> quiet;
    vector<tag_alias_t> alias;
    tag_output_t output;
    tag_input_t input;
}tag_t;

#define PI 3.141592654

//TEST DEFINE
#define VELOCITY_NUM    3
#define CUSTOM_VELOCITY_NUM  3

#ifndef CUSTOM_OFFSET_INDEX
#define CUSTOM_OFFSET_INDEX 0
#define VELOCITY0_SAFETY_DIST 0
#define VELOCITY1_SAFETY_DIST 0
#define VELOCITY2_SAFETY_DIST 0
#endif


//COMMUNICATION RANGE
#define DOWNLINK_MAX_DISTANCE 120
#define UPLINK_MAX_DISTANCE_TYPE1   44
#define UPLINK_MAX_DISTANCE_TYPE2   70
#define UPLINK_MAX_DISTANCE_TYPE3   81

#define UPLINK_MAX_DISTANCE_TYPE3_B125  101
#define UPLINK_MAX_DISTANCE_TYPE3_B250  98
#define UPLINK_MAX_DISTANCE_TYPE3_B500  90

//#define UPLINK_MAX_DISTANCE_TYPE4   110
#define TAG_FOV         40.0       //ref--https://www.hella.com/techworld/us/Technical/Automotive-lighting/LED-headlights-833/#


//ROAD DEFINE
#define MAX_LANE_NUM    3
#define VEHICLE_NUM_PER_LANE    1000
#define LANE_WIDTH      3.5

//ROADSIGN DEFINE
#define TAG_AXIS_NUM    2
#define TAG_POSY_OFFSET 1
#define TAG_SPACING_OFFSET  110
#define INSERT_TAG_INTERVAL 1000

//VEHICLE DEFINE
#define READER_VELOCITY 0.00002  //---m/s
#define READER_NETWORK_SIZE 10

//BITRATE DEFINE 
//#define DOWNLINK_BITRATE    5000
#ifndef DOWNLINK_BITRATE
#define DOWNLINK_BITRATE    5000
#endif
//#define UPLINK_BITRATE  1000
//#define UPLINK_BITRATE  128

//FRAME LENGTH DEFINE
#define DISC_REQUEST_HDR_LEN    7   //include FCS
#define QUERY_REQUEST_HDR_LEN   6
#define QUERY_RESPONSE_HDR_LEN  5

//FRAME FIELD DEFINE
#define DEFAULT_FRAME_TYPE    0x00
#define DISC_REQUEST    0x01
#define DISC_RESPONSE   0x02
#define QUERY_REQUEST   0x03
#define QUERY_RESPONSE  0x04

#define NACK_FLAG    0x00
#define ACK_FLAG    0x08

//TIME DEFINE
#define TIME_INF    0xFFFFFFFFFFFFFFFF
#define DISC_ACK_TIMEOUT 30000
#define PREAMBLE_TIMEOUT 30000
#define ENERGY_TIMEOUT  15000
#define TAG_SOFTWARE_DELAY  0
//#define DOWNLINK_SLOT   (25000*5000/DOWNLINK_BITRATE)
#define DOWNLINK_SLOT   25000
#define ENERGY_CHECK_INTERVAL   1000

#define DOWNLINK_SEND_START 0x0001
#define DOWNLINK_SEND_END   0x0002
#define WAIT_DISC_ACK_TIMEOUT   0x0003
#define WAIT_PREAMBLE_TIMEOUT   0x0004
#define WAIT_PAYLOAD_FINISH     0x0005
#define MAC_READER_END  0x0006

#define UPLINK_SEND_START   0x0011
#define UPLINK_SEND_DISC_ACK_END    0x0012
#define UPLINK_SEND_ENERGY_END  0x0013
#define UPLINK_SEND_PAYLOAD_END 0x0014


const char state_str[21][64] = {
"",
"DOWNLINK_SEND_START",
"DOWNLINK_SEND_END",
"WAIT_DISC_ACK_TIMEOUT" ,
"WAIT_PREAMBLE_TIMEOUT",
"WAIT_PAYLOAD_FINISH",
"MAC_READER_END",
"",
"",
"",
"",
"",
"",
"",
"",
"",
"",
"UPLINK_SEND_START",
"UPLINK_SEND_DISC_ACK_END",
"UPLINK_SEND_ENERGY_END",
"UPLINK_SEND_PAYLOAD_END"
};


uint64_t mac_backoff(){
    //return random() % (READER_NETWORK_SIZE * DOWNLINK_SLOT);
    return random() % READER_NETWORK_SIZE * DOWNLINK_SLOT;
}


uint64_t gen_init_time(){
    return mac_backoff();
}

bool reader_cmp(reader_t &a, reader_t &b){
    return a.init_posx < b.init_posx;
}




double velocity_dist_tbl[VELOCITY_NUM + CUSTOM_VELOCITY_NUM][4]={
    //{kph, m, mph, feet}
//    {48.28032, 40.233},   //30mph, 75feet
//    {80.4672, 67.056},     //50mph
//    {112.65408, 93.875},    //70mph
    {48.28032, 23, 30, 75},   //30mph, 75feet
    {80.4672, 53, 50, 175},     //50mph
    {112.65408, 96, 70, 315},    //70mph     https://en.wikibooks.org/wiki/Driving/Safety/Wet_weather_driving

//CUSTOM TEST
    {48.28032, VELOCITY0_SAFETY_DIST, 30, 75},   //30mph, 75feet
    {80.4672, VELOCITY1_SAFETY_DIST, 50, 175},     //50mph
    {112.65408, VELOCITY2_SAFETY_DIST, 70, 315},    //70mph     https://en.wikibooks.org/wiki/Driving/Safety/Wet_weather_driving
};

uint64_t get_cur_us(){
    struct timeval tv; 
    memset(&tv, 0, sizeof(struct timeval));
    gettimeofday( &tv, NULL );
    uint64_t end_me = 1000000 * tv.tv_sec + tv.tv_usec;
    return end_me;
}


vector<reader_t> readers;
void generate_readers(int n, case_t eval_case){
//    int g_lane_num = 3;
    int g_car_num_per_lane = n / eval_case.lane_num;
//    int g_velocity_type = 2;
    double pos_buf[MAX_LANE_NUM][VEHICLE_NUM_PER_LANE][2];
    for(int i = 0; i < eval_case.lane_num; i++){
        for(int j = 0; j < g_car_num_per_lane; j++){     
            if(j == 0){
                pos_buf[i][j][0] = -(random()%(int)velocity_dist_tbl[eval_case.velocity_type][1]);
                pos_buf[i][j][1] = TAG_POSY_OFFSET + LANE_WIDTH * (i + 0.5);
            }else{
                //pos_buf[i][j][0] = pos_buf[i][0][0];
                pos_buf[i][j][0] = pos_buf[i][j-1][0] - velocity_dist_tbl[eval_case.velocity_type][1];
                pos_buf[i][j][1] = TAG_POSY_OFFSET + LANE_WIDTH * (i + 0.5);
            }
            reader_t reader;
            memset(&reader, 0, sizeof(reader_t));
            reader.send_start_time= gen_init_time();
            reader.send_end_time = TIME_INF;
            reader.check_time = reader.send_start_time;
            reader.state = DOWNLINK_SEND_START;
            reader.init_posx = pos_buf[i][j][0];
            reader.init_posy = pos_buf[i][j][1];
            reader.velocity = velocity_dist_tbl[eval_case.velocity_type][0] * 1000 / 3600 / 1000000;
            reader.max_distance = DOWNLINK_MAX_DISTANCE;
            reader.fov = TAG_FOV/2*PI/180;
            reader.window = 0;
            reader.need_backoff_flag = 0;

            reader.output.type = DEFAULT_FRAME_TYPE;
            reader.output.ack_flag = NACK_FLAG;
            reader.output.round = 0;
            reader.output.collision_num = 0;
            reader.output.overhear_count = 0;
            reader.output.overhear_buf.clear();
            reader.output.overhear_time.clear();
            reader.output.buflen = 0;

            readers.push_back(reader);
        }
    }        
    sort(readers.begin(), readers.end(), reader_cmp);
    for(int i = 0; i < readers.size(); i++){
        readers[i].output.type = DISC_REQUEST;
        readers[i].ID = i + 0x0FF0;
        readers[i].index = i;
        LOG(DEBUG, "readers[%d].init_pos: (%.8f, %.8f)", i, readers[i].init_posx, readers[i].init_posy);
    }
    LOG(DEBUG, "readers generate finish");
}

bool tag_cmp(tag_t &a, tag_t &b){
    return a.init_posx < b.init_posx;
}

vector<tag_t> tags;
void generate_tags(int n, case_t eval_case, int spacing){
    float final_tag_posx = 0;
    for(int i = 0; i < n; i++){
        tag_t tag;
        memset(&tag, 0, sizeof(tag_t));
        tag.send_start_time = TIME_INF;
        tag.send_end_time = TIME_INF;
        tag.check_time = tag.send_start_time;
        tag.state = UPLINK_SEND_START;
        tag.init_posx = TAG_SPACING_OFFSET + i * spacing;
        tag.init_posy = 0;
        tag.max_distance = eval_case.uplink_max_distance;
        tag.fov = TAG_FOV/2*PI/180;        
        tag.uplink_bitrate = eval_case.uplink_bitrate;

        final_tag_posx = tag.init_posx;

        tags.push_back(tag);
	}
    

    tag_t tag;
    float base_posx = TAG_SPACING_OFFSET;
    for(;;){
        if(base_posx > final_tag_posx){
            break;
        }
        for(int j = 0; j < eval_case.collocated_tag_num; j++){
            tag.init_posx = base_posx + random()%INSERT_TAG_INTERVAL;
            tag.max_distance = UPLINK_MAX_DISTANCE_TYPE1;   // 
            tags.push_back(tag);
        }
        base_posx += INSERT_TAG_INTERVAL;
    }

    sort(tags.begin(), tags.end(), tag_cmp);
    for(int i = 0; i < tags.size(); i++){
        tags[i].output.type == DEFAULT_FRAME_TYPE;
        tags[i].ID = i;
        tags[i].index = i;
        tags[i].output.plen = 1;
        memset(tags[i].output.payload, tags[i].ID, tags[i].output.plen);
        LOG(DEBUG, "tags[%d].init_pos: (%.8f, %.8f)", i, tags[i].init_posx, tags[i].init_posy);
    }
    LOG(DEBUG, "tags generate finish");
}


/***************** for downlink ******************/
void get_inrange_tags_downlink(reader_t reader, vector<tag_t> &inrange_tags){
    for(int i = 0; i < tags.size(); i++){
        if(tags[i].init_posx - (reader.init_posx + reader.velocity * reader.send_end_time) > reader.max_distance ||
        tags[i].init_posx - (reader.init_posx + reader.velocity * reader.send_start_time) < 0){
//        if(tags[i].init_posx - (reader.init_posx + reader.velocity * reader.send_end_time) > reader.max_distance){

        }else if(is_connected(reader.init_posx + reader.velocity * reader.send_start_time, reader.init_posy, tags[i].init_posx, tags[i].init_posy, reader.max_distance, reader.fov)){ 
            inrange_tags.push_back(tags[i]);
        }
    }
}

void get_inrange_tags_uplink(reader_t reader, vector<tag_t> &inrange_tags){         //used for carrier sensing.
    for(int i = 0; i < tags.size(); i++){
        if(tags[i].init_posx - (reader.init_posx + reader.velocity * reader.send_start_time) > tags[i].max_distance || 
            tags[i].init_posx - (reader.init_posx + reader.velocity * reader.send_start_time) < 0){
//        if(tags[i].init_posx - (reader.init_posx + reader.velocity * reader.send_start_time) > tags[i].max_distance){         

        }else if(is_connected(reader.init_posx + reader.velocity * reader.send_start_time, reader.init_posy, tags[i].init_posx, tags[i].init_posy, tags[i].max_distance, reader.fov)){ 
            inrange_tags.push_back(tags[i]);
        }
    }
}

int tag_in_range(reader_t reader, tag_t tag, uint64_t start_time, uint64_t end_time){
    double reader_start_posx = reader.init_posx + reader.velocity * start_time;
    double reader_start_posy = reader.init_posy;

    double reader_end_posx = reader.init_posx + reader.velocity * end_time;
    double reader_end_posy = reader.init_posy;
//we suppose that reader could not span the field with a downlink time.
    if(is_connected(reader_start_posx, reader_start_posy, tag.init_posx, tag.init_posy, reader.max_distance, reader.fov)){
        return 1;
    }
    
    if(is_connected(reader_end_posx, reader_end_posy, tag.init_posx, tag.init_posy, reader.max_distance, reader.fov)){
        return 1;
    }
    return 0;
}

int reader_is_blocked(reader_t reader, tag_t tag, uint64_t start_time, uint64_t end_time){
    return 0;
    vector<Point> vehicles; 
    for(int i = reader.index; i < readers.size(); i++){
        double reader_start_posx = readers[i].init_posx + readers[i].velocity * start_time;
        double reader_start_posy = readers[i].init_posy;
        if(reader_start_posx > tag.init_posx){
            break;
        }
        vehicles.push_back(Point(reader_start_posx, reader_start_posy));
    }
    if(vehicles.size() < 2){
        return 0;
    }else{
        if(is_intersect(vehicles, Point(tag.init_posx, tag.init_posy), 0) == false){
            return 0;
        }else{
#if 0        
            LOG(INFO, "SEND READER POS: (%.8f, %.8f)", vehicles[0].x, vehicles[0].y);
            for(int i = 1; i < vehicles.size(); i++){
                LOG(INFO, "(%.8f, %.8f)", vehicles[i].x, vehicles[i].y);
            }
            LOG(INFO, "SIGN POS:(%.8f, %.8f)", tag.init_posx, tag.init_posy);
            assert(0 && "reader is blocked");
#endif            
            return 1;
        }
    }
}

int downlink_collided(collision_t collision, tag_t tag){
    reader_t cur_reader = readers[collision.device_index[0]];
    reader_t collision_reader = readers[collision.device_index[1]];
    if(tag_in_range(collision_reader, tag, collision.start_time, collision.end_time) == 0){ 
        return 0;
    }
    if(reader_is_blocked(collision_reader, tag, collision.start_time, collision.end_time) == 1){
        return 0;    
    }
    return 1;
}


/////////////////// COLLISION CHECK FUNC///////////////////////////
/******************** for uplink ********************/
void get_inrange_readers(tag_t tag, vector<reader_t> &inrange_readers){
    for(int i = 0; i < readers.size(); i++){
        if(tag.init_posx - (readers[i].init_posx + readers[i].velocity * tag.send_end_time) > tag.max_distance ||
        tag.init_posx - (readers[i].init_posx + readers[i].velocity * tag.send_start_time) < 0){
            //could not be in range
        }else{
            if(is_connected(readers[i].init_posx + readers[i].velocity * tag.send_start_time, readers[i].init_posy, tag.init_posx, tag.init_posy, tag.max_distance, readers[i].fov) ||
                is_connected(readers[i].init_posx + readers[i].velocity * tag.send_end_time, readers[i].init_posy, tag.init_posx, tag.init_posy, tag.max_distance, readers[i].fov)){
                inrange_readers.push_back(readers[i]);
            }
        }
    }
}

int reader_in_range(tag_t tag, reader_t reader, uint64_t start_time, uint64_t end_time){
    double reader_start_posx = reader.init_posx + reader.velocity * start_time;
    double reader_start_posy = reader.init_posy;

    double reader_end_posx = reader.init_posx + reader.velocity * end_time;
    double reader_end_posy = reader.init_posy;
//we suppose that reader could not span the field with a downlink time.
    if(is_connected(reader_start_posx, reader_start_posy, tag.init_posx, tag.init_posy, tag.max_distance, reader.fov)){
        return 1;
    }
    
    if(is_connected(reader_end_posx, reader_end_posy, tag.init_posx, tag.init_posy, tag.max_distance, reader.fov)){
        return 1;
    }
    return 0;
}

int tag_is_blocked(tag_t tag, reader_t reader, uint64_t start_time, uint64_t end_time){
    return reader_is_blocked(reader, tag, start_time, end_time);
}

int uplink_collided(collision_t collision, reader_t reader){
    tag_t cur_tag = tags[collision.device_index[0]];
    tag_t collision_tag = tags[collision.device_index[1]];
    if(reader_in_range(collision_tag, reader, collision.start_time, collision.end_time) == 0){
        return 0; 
    }
    if(tag_is_blocked(collision_tag, reader, collision.start_time, collision.end_time) == 1){
        return 0;
    }
    return 1;
}





///////////////////// MAC HANDLE ////////////////////////////////////////////////////////
void reader_enframe(reader_t *reader){
    if(reader->collision_occur){
        reader->output.collision_num++;
        reader->collision_occur = 0;
    }
    reader->output.buf[0] = (reader->ID >> 8) & 0x00FF;
    reader->output.buf[1] = reader->ID & 0x00FF;
    reader->output.dstAddr++;
    reader->output.buf[2] = ((reader->output.type | reader->output.ack_flag) << 4) + reader->output.dstAddr;
    reader->output.buf[4] = reader->output.ack_crc8;
    int link_hdr_len = 0;
    if(reader->output.type == DISC_REQUEST){

        reader->output.round++;
        reader->output.buf[3] = reader->output.round;
        link_hdr_len = 6;
        reader->output.buf[5] = (reader->output.collision_num << 4) + reader->output.overhear_count;
        if(reader->output.collision_num){
            reader->window = 2 * reader->output.collision_num; 
        }else{
            reader->window = 1;
        }
        int frame_buff_len = 6;
        int tmp_buf_index = 0;
        for(int i = 0; i < reader->output.overhear_count; i++){
            reader->output.buf[frame_buff_len++] = reader->output.overhear_buf[tmp_buf_index++];
            reader->output.buf[frame_buff_len++] = reader->output.overhear_buf[tmp_buf_index++];
            reader->output.buf[frame_buff_len++] = reader->output.overhear_buf[tmp_buf_index++];
            reader->output.buf[frame_buff_len++] = reader->output.overhear_buf[tmp_buf_index++];
            if(i % 2 == 0){
                reader->output.buf[link_hdr_len + reader->output.overhear_count * 4 + i/2] = reader->output.overhear_buf[tmp_buf_index++] << 4;
            }else{
                reader->output.buf[link_hdr_len + reader->output.overhear_count * 4 + i/2] += reader->output.overhear_buf[tmp_buf_index++] & 0x0F;
            }
        }
        reader->output.buflen = link_hdr_len + (reader->output.overhear_count * 9 + 1) / 2;
//after sending discovery request, update reader output
        reader->output.dstAddr = 0;
        reader->output.ack_flag = NACK_FLAG;
        LOG(DEBUG, "reader output round:%d", reader->output.round);
        reader->output.ack_crc8 = 0;
        reader->output.collision_num = 0;
        reader->output.overhear_count = 0;
        reader->output.overhear_buf.clear();
        reader->output.overhear_time.clear();

    }else if(reader->output.type == QUERY_REQUEST){
        reader->output.buf[3] = reader->output.round;
        link_hdr_len = 5;
        reader->output.buflen = link_hdr_len;
    }
    reader->output.buf[reader->output.buflen] = CRC8_GEN(reader->output.buf, reader->output.buflen);
    reader->output.buflen++;
}

void reader_deframe(reader_t *reader){
    reader->input.dstAddr = reader->input.buf[0] << 8;
    reader->input.dstAddr += reader->input.buf[1];
    reader->input.srcAddr = reader->input.buf[2] >> 4;
    reader->input.plen = reader->input.buf[2] & 0x0F;
    reader->input.round = reader->input.buf[3];
    memcpy(reader->input.payload, &reader->input.buf[4], reader->input.plen);
}


uint8_t tag_enframe(tag_t *tag){
    tag->output.buf[0] = tag->output.dstAddr >> 8;
    tag->output.buf[1] = tag->output.dstAddr & 0x00FF;
    tag->output.buf[2] = tag->output.srcAddr << 4;
    tag->output.buf[2] += tag->output.plen;
    tag->output.buf[3] = tag->output.round;
    memcpy(&tag->output.buf[4], tag->output.payload, tag->output.plen);
    tag->output.buflen = 4 + tag->output.plen;
    tag->output.buf[tag->output.buflen] = CRC8_GEN(tag->output.buf, tag->output.buflen);
    tag->output.buflen++;
    return CRC8_GEN(&tag->output.buf[4], tag->output.plen);
}

void tag_deframe(tag_t *tag){
    tag->input.srcAddr = (tag->input.buf[0] << 8) + tag->input.buf[1];
    tag->input.type = ((tag->input.buf[2] >> 4) & 0x07);
    tag->input.ack_flag = ((tag->input.buf[2] >> 4) & 0x08);
    LOG(DEBUG, "tag[%d]->input.ack_flag:%02X", tag->index, tag->input.ack_flag);
    tag->input.dstAddr = tag->input.buf[2] & 0x0F;
    tag->input.round = tag->input.buf[3];
    tag->input.ack_crc8 = tag->input.buf[4];
    if(tag->input.type == DISC_REQUEST){
        tag->input.collision_num = tag->input.buf[5] >> 4;
        tag->input.overhear_count = tag->input.buf[5] & 0x0F;
        tag->input.overhear_buf.clear();
        for(int i = 0; i < tag->input.overhear_count; i++){
            tag->input.overhear_buf.push_back(tag->input.buf[6 + i * 4 + 0]);
            tag->input.overhear_buf.push_back(tag->input.buf[6 + i * 4 + 1]);
            tag->input.overhear_buf.push_back(tag->input.buf[6 + i * 4 + 2]);
            tag->input.overhear_buf.push_back(tag->input.buf[6 + i * 4 + 3]);
            if(i % 2 == 0){
                tag->input.overhear_buf.push_back(tag->input.buf[6 + tag->input.overhear_count * 4 + i / 2] >> 4);
            }else{
                tag->input.overhear_buf.push_back(tag->input.buf[6 + tag->input.overhear_count * 4 + i / 2] & 0x0F);
            }
        }
//        LOG(INFO, "tags[%d].overhear_buf.size():%ld", tag->index, tag->input.overhear_buf.size());
#if 0        
        for(int i = 0; i < tag->input.overhear_buf.size(); i++){
            printf("%02X ", tag->input.overhear_buf[i]);
        }
        printf("\n");
#endif        
    }
}

int tag_check_ack_crc8(tag_t *tag, uint32_t alias, uint8_t ack_crc8){
    for(int i = 0; i < tag->alias.size(); i++){
#if 0 
        LOG(INFO, "tags[%d]->alias[%d].addr:%08X, alias:%08X", tag->index, i, tag->alias[i].addr, alias);
        LOG(INFO, "tags[%d]->alias[%d].sent:%d", tag->index, i, tag->alias[i].sent);
        LOG(INFO, "tags[%d]->alias[%d].ack_crc8:%d, ack_crc8:%d", tag->index, i, tag->alias[i].ack_crc8, ack_crc8);
#endif        
        if(tag->alias[i].addr == alias &&
        tag->alias[i].sent &&
        tag->alias[i].ack_crc8 == ack_crc8){
            LOG(DEBUG, "tags[%d] check ack crc8 successful", tag->index);
            return 1;
        }
    }
//    LOG(INFO,"tags[%d] check ack crc8 failed", tag->index);
    return 0;
}

void tag_set_disc_quiet(tag_t *tag, uint16_t reader_addr){
    for(int i = 0; i < tag->quiet.size(); i++){
        if(reader_addr == tag->quiet[i].addr){
            return;
        }
    }
    tag_quiet_t quiet;
    quiet.addr = reader_addr;
    tag->quiet.push_back(quiet);
}

void tag_set_reader_quiet(tag_t *tag){
//    LOG(INFO, "tag[%d]->input.overhear_count:%d", tag->index, tag->input.overhear_count);
#if 0    
    for(int i = 0; i < tag->input.overhear_buf.size(); i++){
        printf("%02X ", tag->input.overhear_buf[i]);
    }
    printf("\n");
#endif
    for(int i = 0; i < tag->input.overhear_count; i++){    
        uint32_t alias = 0;    
        alias = tag->input.overhear_buf[i*5];
        alias <<= 8;
        alias += tag->input.overhear_buf[i*5 + 1]; 
        alias <<= 8;
        alias += tag->input.overhear_buf[i*5 + 2];    
        alias <<= 8;
        uint8_t ack_crc8 = tag->input.overhear_buf[i*5 + 3]; 
        alias += (tag->input.overhear_buf[i*5 + 4] << 4);
        alias >>= 4;

//        LOG(INFO, "check alias:%08X", alias);
        if(tag_check_ack_crc8(tag, alias, ack_crc8)){
            tag_set_disc_quiet(tag, tag->input.srcAddr);
            LOG(DEBUG, "set reader.addr=%d to disc quiet", tag->input.srcAddr);
        }
    }
}

void tag_add_alias(tag_t *tag, uint32_t tag_addr){
//    LOG(DEBUG, "tag[%d].alias.addr:%08X", tag->index, tag_addr);
    for(int i = 0; i < tag->alias.size(); i++){
        if(tag->alias[i].addr == tag_addr){
            return;
        }
    }
    tag_alias_t tag_alias;
    tag_alias.addr = tag_addr;
    tag_alias.sent = 0;
    tag_alias.ack_crc8 = 0;
    tag->alias.push_back(tag_alias);
}


void tag_set_alias_ack_crc8(tag_t *tag, uint32_t alias, uint8_t ack_crc8){
    for(int i = 0; i < tag->alias.size(); i++){
        if(tag->alias[i].addr == alias){
            tag->alias[i].sent = 1;
            tag->alias[i].ack_crc8 = ack_crc8;
        }
    }
}

int tag_check_alias(tag_t *tag, uint32_t alias){
    for(int i = 0; i < tag->alias.size(); i++){
        if(tag->alias[i].addr == alias){
            return 1;
        }
    }
    return 0;
}

int tag_check_reader_quiet(tag_t *tag, uint32_t reader_addr){
    for(int i = 0; i < tag->quiet.size(); i++){
        if(tag->quiet[i].addr == reader_addr){
            return 1;
        }
    }
    return 0;
}


void tag_handle(tag_t *tag, reader_t *reader){
    if((tag->input.type & 0x07) == DISC_REQUEST){
        if(tag->input.overhear_count){     //handle overhearing ack
            tag_set_reader_quiet(tag);
        }
        if(tag->input.ack_flag == ACK_FLAG){ //handle listening ack
            uint32_t alias = 0;
            alias = tag->input.srcAddr;
            alias <<= 8;
            alias += tag->input.round - 1;
            alias <<= 4;
            alias += tag->input.dstAddr - 1;
            if(tag_check_ack_crc8(tag, alias, tag->input.ack_crc8)){
                tag_set_disc_quiet(tag, tag->input.srcAddr);
            }
        }
        if(tag_check_reader_quiet(tag, tag->input.srcAddr) == 0){    //assign
            uint8_t shortAddr = 0;
            uint32_t alias = 0;
            alias = tag->input.srcAddr;
            alias <<= 8;
            alias += tag->input.round;
            alias <<= 4;
            uint8_t window = 0;
            if(tag->input.collision_num){
                window = 2 * tag->input.collision_num;
            }else{
                window = 1;
            }
            shortAddr = random() % window;
            alias += shortAddr;
            tag_add_alias(tag, alias);
            tag->output.dstAddr = tag->input.srcAddr;
            tag->output.round = tag->input.round;
            tag->output.srcAddr = shortAddr;
            LOG(DEBUG, "DISC RESPONSE, tag[%d].srcAddr:%d", tag->index, tag->output.srcAddr);
        }else{
            tag->output.srcAddr = -1;
        }
        tag->output.type = DISC_RESPONSE;
        LOG(DEBUG, "readers[%d] finish sending DISC REQUEST to tags[%d] at %ld, shortAddr:%d", reader->index, tag->index, reader->send_end_time, tag->output.srcAddr);
    }else if((tag->input.type & 0x07) == QUERY_REQUEST){
        LOG(DEBUG, "tag received type: QUERY_REQUEST");
        uint32_t alias = 0;
        alias = tag->input.srcAddr;
        alias <<= 8;
        alias += tag->input.round;
        alias <<= 4;
        alias += tag->input.dstAddr;
//        LOG(INFO, "alias:%08X", alias);
        if(tag->input.ack_flag == ACK_FLAG){ //handle listening ack
            LOG(DEBUG, "tag[%d] recvd ack flag", tag->index);
            if(tag_check_ack_crc8(tag, alias - 1, tag->input.ack_crc8)){
                LOG(DEBUG, "set reader quiet");
                tag_set_disc_quiet(tag, tag->input.srcAddr);
            }
        }
//        LOG(DEBUG, "recvd alias:%08X", alias);
//        int a = tag_check_alias(tag, alias);
//        int b = tag_check_reader_quiet(tag, tag->input.srcAddr);
//        LOG(DEBUG, "a:%d, b:%d, tag->alias.size:%ld, last_alias:%08X", a,b, tag->alias.size(), tag->alias.back().addr);
        if(tag_check_alias(tag, alias) && (tag_check_reader_quiet(tag, tag->input.srcAddr) == 0)){
//        if(a && (b == 0)){
            tag->output.dstAddr = tag->input.srcAddr;
            tag->output.srcAddr = tag->input.dstAddr;
            tag->output.round = tag->input.round;
            tag->output.type = QUERY_RESPONSE;
            LOG(DEBUG, "readers[%d] finish sending QUERY REQUEST to tag[%d] at %ld", reader->index, tag->index, reader->send_end_time);
        }else{
 //           LOG(DEBUG, "maybe reader is quiet?");
        }
    }
}

int downlink_send_success_count = 0; 
int downlink_send_fail_count = 0; 


void reader_send_downlink(reader_t *reader, tag_t *tag){
    //send to tag
    if(tag->state == UPLINK_SEND_START && tag->check_time == TIME_INF){
        downlink_send_success_count++;
        memcpy(tag->input.buf, reader->output.buf, reader->output.buflen);
        tag->input.buflen = reader->output.buflen;

        LOG(DEBUG, "before tags[%d].output.type:%d", tag->index, tag->output.type);
        tag_deframe(tag);
#if 0    
        if(tag->input.type == DISC_REQUEST){
            LOG(INFO, "readers[%d] send DISC_REQUEST to tags[%d]", reader->index, tag->index);
        }else if(tag->input.type == QUERY_REQUEST){
            LOG(INFO, "readers[%d] send QUERY_REQUEST to tags[%d]", reader->index, tag->index);
        }
#endif    
        tag_handle(tag, reader);
        if(tag->output.type == QUERY_RESPONSE || tag->output.type == DISC_RESPONSE){
            tag->state = UPLINK_SEND_START;
            tag->check_time = reader->check_time;
            //        LOG(DEBUG, "index:%d check time:%ld", tag->index, tag->check_time);
        }
        LOG(DEBUG, "after tags[%d].output.type:%d", tag->index, tag->output.type);
    }else{
        downlink_send_fail_count++;
    }
}

int reader_check_next_addr_available(reader_t *reader){
    if(reader->output.dstAddr + 1 < reader->window){
        return 1;
    }else{
        return 0;
    }
}

void reader_query_next(reader_t *reader){
    if(reader_check_next_addr_available(reader)){
        reader->output.type = QUERY_REQUEST;
        reader->state = DOWNLINK_SEND_START;
        reader->check_time += 0; 
    }else{
        reader->output.type = DISC_REQUEST;
        reader->state = DOWNLINK_SEND_START;
        reader->check_time += mac_backoff();
    }
}


int reader_record_tagID(reader_t *reader, uint8_t tagID){
    LOG(DEBUG, "TAG ID:%d", tagID);
    for(int i = 0; i < reader->tagID.size(); i++){
        LOG(DEBUG, "readers[%d]->tagID[%d]:%d, tagID:%d", reader->index, i, reader->tagID[i], tagID);
        if(reader->tagID[i] == tagID){
            return 0;
        }
    }
    reader->tagID.push_back(tagID);
    return 1;
}


void tag_send_ack(tag_t *tag, reader_t *reader){
    if(reader->state == WAIT_DISC_ACK_TIMEOUT){
        reader->state = WAIT_PREAMBLE_TIMEOUT;
        reader->check_time = tag->check_time + PREAMBLE_TIMEOUT + TAG_SOFTWARE_DELAY;
    }else if(reader->state == WAIT_PREAMBLE_TIMEOUT){
        if(reader->check_time < tag->send_end_time){
            reader->state = WAIT_PAYLOAD_FINISH;
            reader->check_time = tag->check_time + ((tag->output.plen + QUERY_RESPONSE_HDR_LEN)<< 3) * 1000000 / tag->uplink_bitrate + PREAMBLE_TIMEOUT - ENERGY_TIMEOUT ;
        }
    }
    LOG(DEBUG, "tags[%d] send ack to readers[%d] reader_state:%s", tag->index, reader->index, state_str[reader->state]);
    tag->output.type == DEFAULT_FRAME_TYPE;
}

void tag_send_preamble_energy(tag_t *tag, reader_t *reader){
    if(reader->state == WAIT_PREAMBLE_TIMEOUT){
        reader->state = WAIT_PAYLOAD_FINISH;
        reader->check_time = tag->send_end_time;
    }else if(reader->state == WAIT_DISC_ACK_TIMEOUT){
        if(reader->check_time < tag->send_end_time){
            reader->state = WAIT_PREAMBLE_TIMEOUT;
            reader->check_time = tag->check_time + PREAMBLE_TIMEOUT + TAG_SOFTWARE_DELAY;
        }
    }
    LOG(DEBUG, "tags[%d] send preamble energy to readers[%d] reader_state:%s, tag->check_time:%ld, tag->send_end_time:%ld", tag->index, reader->index, state_str[reader->state], tag->check_time, tag->send_end_time);
}


int uplink_send_fail_count[60] = {0};
int uplink_send_success_count[60] = {0};

void tag_send_collided_payload(tag_t *tag, reader_t *reader){
	uplink_send_fail_count[reader->index]++;
    if(reader->state == WAIT_PAYLOAD_FINISH){
        reader->state = MAC_READER_END;
        reader->collision_occur++;
        reader->output.ack_flag = NACK_FLAG;
        if(reader_check_next_addr_available(reader)){
            reader->output.type = QUERY_REQUEST;
            reader->state = DOWNLINK_SEND_START;
            reader->check_time += 0;
        }
    }else if(reader->state == WAIT_DISC_ACK_TIMEOUT){
        if(reader->check_time < tag->send_end_time){
            reader->state = WAIT_PREAMBLE_TIMEOUT;
            reader->check_time = tag->check_time + PREAMBLE_TIMEOUT + TAG_SOFTWARE_DELAY;
        }
    }else if(reader->state == WAIT_PREAMBLE_TIMEOUT){
        if(reader->check_time < tag->send_end_time){
            reader->state = WAIT_PAYLOAD_FINISH;
            reader->check_time = tag->check_time + ((tag->output.plen + QUERY_RESPONSE_HDR_LEN)<< 3) * 1000000 / tag->uplink_bitrate + PREAMBLE_TIMEOUT - ENERGY_TIMEOUT ;
        }
    }
    LOG(DEBUG, "######tags[%d] send collided payload to readers[%d] reader_state:%s", tag->index, reader->index, state_str[reader->state]);
    tag->output.type == DEFAULT_FRAME_TYPE;
}

void tag_send_normal_payload(tag_t *tag, reader_t *reader){
	uplink_send_success_count[reader->index]++;
//    assert(0 && "stop here");
    tag_enframe(tag);
    uint32_t alias = tag->output.dstAddr;
    alias <<= 8; 
    alias += tag->output.round;
    alias <<= 4;
    alias += tag->output.srcAddr;
    uint8_t ack_crc8 = CRC8_GEN(tag->output.payload, tag->output.plen);
    tag_set_alias_ack_crc8(tag, alias, ack_crc8);
    if(reader->state == WAIT_PAYLOAD_FINISH){
        LOG(DEBUG, "==============tags[%d] send normal listen data to readers[%d]", tag->index, reader->index);        
        reader->state = MAC_READER_END;
        memcpy(reader->input.buf, tag->output.buf, tag->output.buflen);            
        reader_deframe(reader);        
        reader_record_tagID(reader, reader->input.payload[0]);
        reader->output.ack_flag = ACK_FLAG;
        reader->output.ack_crc8 = CRC8_GEN(reader->input.payload, reader->input.plen);
        if(reader_check_next_addr_available(reader)){
            reader->output.type = QUERY_REQUEST;
            reader->state = DOWNLINK_SEND_START;
            reader->check_time += 0;
        }
    }else if(reader->state == DOWNLINK_SEND_START){
        if(reader->check_time < tag->send_end_time){
            reader->check_time = tag->send_end_time + mac_backoff();
        }else{
            LOG(DEBUG, "==============tags[%d] send normal overhear data to readers[%d]", tag->index, reader->index);
            memcpy(reader->input.buf, tag->output.buf, tag->output.buflen);            
            reader_deframe(reader);        
            if(reader_record_tagID(reader, reader->input.payload[0])){
                reader->output.overhear_buf.push_back(reader->input.dstAddr >> 8);
                reader->output.overhear_buf.push_back(reader->input.dstAddr & 0x00FF);
                reader->output.overhear_buf.push_back(reader->input.round);
                reader->output.overhear_buf.push_back(CRC8_GEN(reader->input.payload, reader->input.plen));
                reader->output.overhear_buf.push_back(reader->input.srcAddr);
                reader->output.overhear_time.push_back(tag->check_time);
                reader->output.overhear_count++;
            }
        }
    }else{
        if(reader->state == WAIT_DISC_ACK_TIMEOUT){
            if(reader->check_time < tag->send_end_time){
                reader->state = WAIT_PREAMBLE_TIMEOUT;
                reader->check_time = tag->check_time + PREAMBLE_TIMEOUT + TAG_SOFTWARE_DELAY;
            }
        }else if(reader->state == WAIT_PREAMBLE_TIMEOUT){
            if(reader->check_time < tag->send_end_time){
                reader->state = WAIT_PAYLOAD_FINISH;
                reader->check_time = tag->check_time + ((tag->output.plen + QUERY_RESPONSE_HDR_LEN)<< 3) * 1000000 / tag->uplink_bitrate + PREAMBLE_TIMEOUT - ENERGY_TIMEOUT;
            }
        }
        if(reader->state != DOWNLINK_SEND_END){
            LOG(DEBUG, "==============tags[%d] send normal overhear data to readers[%d]", tag->index, reader->index);
            memcpy(reader->input.buf, tag->output.buf, tag->output.buflen);            
            reader_deframe(reader);        
            if(reader_record_tagID(reader, reader->input.payload[0])){
                reader->output.overhear_buf.push_back(reader->input.dstAddr >> 8);
                reader->output.overhear_buf.push_back(reader->input.dstAddr & 0x00FF);
                reader->output.overhear_buf.push_back(reader->input.round);
                reader->output.overhear_buf.push_back(CRC8_GEN(reader->input.payload, reader->input.plen));
                reader->output.overhear_buf.push_back(reader->input.srcAddr);
                reader->output.overhear_time.push_back(tag->check_time);
                reader->output.overhear_count++;
            }
        }
    }
    tag->output.type == DEFAULT_FRAME_TYPE;
}


int uplink_is_busy(reader_t reader){
    uint64_t start_t = get_cur_us();
    vector<tag_t> inrange_tags;
    get_inrange_tags_uplink(reader, inrange_tags);
    test_counter += (get_cur_us() - start_t);
    for(int i = 0; i < inrange_tags.size(); i++){
        if(reader.send_start_time >= inrange_tags[i].send_start_time &&
            reader.send_start_time < inrange_tags[i].send_end_time){
            return 1;
        }
    }
    return 0;
}

void tag_block_reader(tag_t tag){
    vector<reader_t> inrange_readers;
    get_inrange_readers(tag, inrange_readers);
    for(int i = 0; i < inrange_readers.size(); i++){
        if(inrange_readers[i].send_start_time >= tag.send_start_time && 
            inrange_readers[i].send_start_time < tag.send_end_time){
            readers[inrange_readers[i].index].check_time = tag.send_end_time + mac_backoff();
            readers[inrange_readers[i].index].send_start_time = readers[inrange_readers[i].index].check_time;
        }
    }
}


void update_overhear(reader_t *reader){
    for(int i = reader->output.overhear_count-1; i >= 0; i--){
        if((reader->check_time - reader->output.overhear_time[i]) * reader->velocity > reader->max_distance){
            reader->output.overhear_time.erase(reader->output.overhear_time.begin()+i);
            reader->output.overhear_buf.erase(reader->output.overhear_buf.begin()+5*(i), reader->output.overhear_buf.begin()+5*(i+1)); 
        }
    }
    reader->output.overhear_count = reader->output.overhear_time.size();
}






//for downlink:
//1.start send time
//2.end send time
//3.end send time + ack time
//4.end send time + ack time + preamble time
//5.end send time + preamble time
//
//for uplink:
//start send time
//start send time + ack time + preamble time
//start send time + preamble time
//end send time
//
//
//update the collision at the begin of sending.
//update the collision and each peer device at the end of sending.
void update_device(int check_index, int check_state){

//    usleep(1000);
#if 0    
    if(check_state!=DOWNLINK_SEND_START){    
        LOG(INFO, "cur state:%s", state_str[check_state]);
    }
#endif    
    switch(check_state){
        /**************************FOR READER****************************/
        case DOWNLINK_SEND_START:
            {
                readers[check_index].send_start_time = readers[check_index].check_time;
                if(uplink_is_busy(readers[check_index])){
                    readers[check_index].check_time += ENERGY_CHECK_INTERVAL;
                    readers[check_index].send_start_time += ENERGY_CHECK_INTERVAL;
                    readers[check_index].need_backoff_flag = 1;
                }else{
                    if(readers[check_index].need_backoff_flag){
                        uint64_t backoff_time = mac_backoff();
                        readers[check_index].check_time += backoff_time;
                        readers[check_index].send_start_time += backoff_time;
                        readers[check_index].need_backoff_flag = 0;
                    }else{
                        //change to next check state, and update the check time.
                        if(readers[check_index].output.type == DISC_REQUEST){
                            update_overhear(&readers[check_index]);
                            readers[check_index].state = DOWNLINK_SEND_END;
                            readers[check_index].check_time += ((DISC_REQUEST_HDR_LEN + (readers[check_index].output.overhear_count * 9 + 1) / 2) << 3) * 1000000 / DOWNLINK_BITRATE;
                        }else if(readers[check_index].output.type == QUERY_REQUEST){
                            readers[check_index].state = DOWNLINK_SEND_END;
                            readers[check_index].check_time += ((QUERY_REQUEST_HDR_LEN) << 3)*1000000/DOWNLINK_BITRATE;
                        }
                        readers[check_index].send_end_time = readers[check_index].check_time;

                        //update downlink collision(potenial collision)
                        for(int i = 0; i < readers.size(); i++){
                            if(i != check_index &&
                                    (readers[check_index].send_start_time > readers[i].send_start_time || (readers[check_index].send_start_time == readers[i].send_start_time && check_index > i))&&
                                    readers[check_index].send_start_time < readers[i].send_end_time){
                                collision_t downlink_collision;
                                downlink_collision.start_time = readers[i].send_start_time;
                                //for reader[check_index]
                                downlink_collision.device_index[0] = check_index;
                                downlink_collision.device_index[1] = i;
                                readers[check_index].collisions.push_back(downlink_collision);
                                //for reader[i]
                                downlink_collision.device_index[1] = check_index;
                                downlink_collision.device_index[0] = i;
                                readers[i].collisions.push_back(downlink_collision);                        
                            }                    
                        }
                    }
                }
            }
            break;

        case DOWNLINK_SEND_END:
            {
                uint64_t device_check_time = readers[check_index].check_time; 
                vector<tag_t> inrange_tags;
                get_inrange_tags_downlink(readers[check_index], inrange_tags);
                for(int i = readers[check_index].collisions.size() - 1; i >= 0; i--){   //for each potenial collision.
                    readers[check_index].collisions[i].end_time = device_check_time;
                    for(int j = inrange_tags.size() - 1; j >= 0; j--){
                        if(downlink_collided(readers[check_index].collisions[i], inrange_tags[j])){
                            //if downlink is collided, send nothing. erase the tag from inrange_tags
                            inrange_tags.erase(inrange_tags.begin()+j);
                            LOG(DEBUG, "downlink collided");
                        }
                    }

                    //update collisions
                    int collision_reader_index = readers[check_index].collisions[i].device_index[1];
                    for(int k = 0; k < readers[collision_reader_index].collisions.size(); k++){
                        if(readers[collision_reader_index].collisions[k].device_index[1] == check_index &&
                                readers[collision_reader_index].collisions[k].start_time == readers[check_index].collisions[i].start_time){
                            readers[collision_reader_index].collisions[k].end_time = device_check_time;
                        }
                    }
                    readers[check_index].collisions.erase(readers[check_index].collisions.begin() + i);
                }

                int reader_enframed_flag = 0;
                if(inrange_tags.size()){
                    for(int j = 0; j < inrange_tags.size(); j++){
                        //no collision happened, but maybe have a blockage.
                        if(reader_is_blocked(readers[check_index], inrange_tags[j], readers[check_index].send_start_time, readers[check_index].send_end_time) == 0){
                            if(reader_enframed_flag == 0){
                                reader_enframed_flag = 1;
                                reader_enframe(&readers[check_index]);
                            }
                            reader_send_downlink(&readers[check_index], &tags[inrange_tags[j].index]);
                        }
                    }
                }else{
                    if(readers[check_index].output.type == QUERY_REQUEST){
                        reader_enframe(&readers[check_index]);
                    }    
                }

                if(readers[check_index].output.type == DISC_REQUEST){
                    readers[check_index].state = WAIT_DISC_ACK_TIMEOUT;
                    readers[check_index].check_time += DISC_ACK_TIMEOUT + TAG_SOFTWARE_DELAY;
                }else if(readers[check_index].output.type == QUERY_REQUEST){
                    readers[check_index].state = WAIT_PREAMBLE_TIMEOUT;
                    readers[check_index].check_time += PREAMBLE_TIMEOUT + TAG_SOFTWARE_DELAY;
                }
            }
            break;

        case WAIT_DISC_ACK_TIMEOUT:
            {
                //timeout, downlink collided or there is no any tag in-view.
                readers[check_index].state = DOWNLINK_SEND_START;
                readers[check_index].check_time += mac_backoff();
            }
            break;
        case WAIT_PREAMBLE_TIMEOUT:
        case WAIT_PAYLOAD_FINISH:   //actually, it never happen.
        case MAC_READER_END:
            {
//                assert(0&&"stop here");
                reader_query_next(&readers[check_index]);
            }
            break;


            /**************************FOR TAG****************************/
        case UPLINK_SEND_START:
            {
                LOG(DEBUG, "tag[%d] output type:%d", check_index, tags[check_index].output.type);
                tags[check_index].send_start_time = tags[check_index].check_time;
                if(tags[check_index].output.type == DISC_RESPONSE){  //send disc ack.
                    tags[check_index].state = UPLINK_SEND_DISC_ACK_END;
                    tags[check_index].check_time += DISC_ACK_TIMEOUT;
                    tags[check_index].send_end_time = tags[check_index].check_time;
                }else if(tags[check_index].output.type == QUERY_RESPONSE){   //send preamble + payload.
                    tags[check_index].state = UPLINK_SEND_ENERGY_END;
                    tags[check_index].check_time += ENERGY_TIMEOUT;
                    tags[check_index].send_end_time = tags[check_index].send_start_time + ((tags[check_index].output.plen + QUERY_RESPONSE_HDR_LEN)<< 3) * 1000000 / tags[check_index].uplink_bitrate + PREAMBLE_TIMEOUT - ENERGY_TIMEOUT ;
                }

                //tag_block_reader(tags[check_index]);

                for(int i = 0; i < tags.size(); i++){
                    if(i != check_index && 
                            tags[check_index].send_start_time >= tags[i].send_start_time && 
                            tags[check_index].send_start_time < tags[i].send_end_time){
                        //save potenial collision.
                        collision_t uplink_collision;
                        uplink_collision.start_time = tags[check_index].send_start_time;
                        //for tag[check_index]
                        uplink_collision.device_index[0] = check_index;
                        uplink_collision.device_index[1] = i;
                        tags[check_index].collisions.push_back(uplink_collision);

                        //for tag[i]
                        uplink_collision.device_index[0] = i;
                        uplink_collision.device_index[1] = check_index;
                        tags[i].collisions.push_back(uplink_collision);
                    }
                }
            }
            break;

        case UPLINK_SEND_DISC_ACK_END:
            {
                LOG(DEBUG, "tag[%d].state:%s", check_index, state_str[tags[check_index].state]);
                uint64_t device_check_time = tags[check_index].check_time; 
                vector<reader_t> inrange_readers;
                get_inrange_readers(tags[check_index], inrange_readers);

                //collision first
                for(int i = tags[check_index].collisions.size() - 1; i >= 0; i--){      //for each potential collision. 
                    tags[check_index].collisions[i].end_time = device_check_time;
                    for(int j = inrange_readers.size() - 1; j >= 0; j--){                 //for each in-view reader.                    
                        //check collision
                        if(uplink_collided(tags[check_index].collisions[i], inrange_readers[j])){
                            //uplink collide. 
                            tag_send_ack(&tags[check_index], &readers[inrange_readers[j].index]);
                            inrange_readers.erase(inrange_readers.begin() + j);
                        }
                    }

                    //update collisions
                    int collision_tag_index = tags[check_index].collisions[i].device_index[1];
                    for(int k = 0; k < tags[collision_tag_index].collisions.size(); k++){
                        if(tags[collision_tag_index].collisions[k].device_index[1] == check_index && 
                                tags[collision_tag_index].collisions[k].start_time == tags[check_index].collisions[i].start_time){
                            tags[collision_tag_index].collisions[k].end_time = device_check_time; 
                        }
                    }
                    tags[check_index].collisions.erase(tags[check_index].collisions.begin() + i);
                }
                for(int j = 0; j < inrange_readers.size(); j++){
                    //no collision happened, but maybe have a blockage.
                    if(tag_is_blocked(tags[check_index], inrange_readers[j], tags[check_index].send_start_time, tags[check_index].send_end_time) == 0){
                        tag_send_ack(&tags[check_index], &readers[inrange_readers[j].index]);
                    }
                }

                if(tags[check_index].output.srcAddr == 0){     //send preamble and payload following disc ack response.
                    tags[check_index].output.type = QUERY_RESPONSE;
                    tags[check_index].state = UPLINK_SEND_START;
                    tags[check_index].check_time = device_check_time;
                }else{
                    tags[check_index].output.type = DEFAULT_FRAME_TYPE;
                    tags[check_index].state = UPLINK_SEND_START;
                    tags[check_index].check_time = TIME_INF;
                }
            }
            break;

        case UPLINK_SEND_ENERGY_END:
            {
                LOG(DEBUG, "tag[%d].state:%s", check_index, state_str[tags[check_index].state]);
                uint64_t device_check_time = tags[check_index].check_time; 
                vector<reader_t> inrange_readers;
                get_inrange_readers(tags[check_index], inrange_readers);
                for(int j = 0; j < inrange_readers.size(); j++){
                    //no collision happened, but maybe have a blockage.
                    if(tag_is_blocked(tags[check_index], inrange_readers[j], tags[check_index].send_start_time, tags[check_index].send_end_time) == 0){
                        tag_send_preamble_energy(&tags[check_index], &readers[inrange_readers[j].index]); 
                    }
                }
                tags[check_index].state = UPLINK_SEND_PAYLOAD_END;
                tags[check_index].check_time = tags[check_index].send_end_time;
            }
            break;

        case UPLINK_SEND_PAYLOAD_END:
            {
                uint64_t device_check_time = tags[check_index].check_time; 
                vector<reader_t> inrange_readers;
                get_inrange_readers(tags[check_index], inrange_readers);
                for(int i = tags[check_index].collisions.size() - 1; i >= 0; i--){
                    tags[check_index].collisions[i].end_time = device_check_time;
                    for(int j = inrange_readers.size() - 1; j >= 0; j--){
                        //check collision
                        if(uplink_collided(tags[check_index].collisions[i], inrange_readers[j])){
                            //uplink collide. 
                            LOG(DEBUG, "collision[%d].device_index[0]:%d", i, tags[check_index].collisions[i].device_index[0]);
                            LOG(DEBUG, "collision[%d].device_index[1]:%d", i, tags[check_index].collisions[i].device_index[1]);
                            LOG(DEBUG, "collision[%d].start_time:%ld", i, tags[check_index].collisions[i].start_time);
                            LOG(DEBUG, "collision[%d].end_time:%ld", i, tags[check_index].collisions[i].end_time);
                            tag_send_collided_payload(&tags[check_index], &readers[inrange_readers[j].index]);
                            inrange_readers.erase(inrange_readers.begin() + j);
                        }
                    }

                    //update collisions
                    tags[check_index].collisions.erase(tags[check_index].collisions.begin() + i);
                    int collision_tag_index = tags[check_index].collisions[i].device_index[1];
                    for(int k = 0; k < tags[collision_tag_index].collisions.size(); k++){
                        if(tags[collision_tag_index].collisions[k].device_index[1] == check_index && 
                                tags[collision_tag_index].collisions[k].start_time == tags[check_index].collisions[i].start_time){
                            tags[collision_tag_index].collisions[k].end_time = device_check_time; 
                        }
                    }
                }
                for(int j = 0; j < inrange_readers.size(); j++){
                    //no collision happened, but maybe have a blockage.
                    if(tag_is_blocked(tags[check_index], inrange_readers[j], tags[check_index].send_start_time, tags[check_index].send_end_time) == 0){
                        tag_send_normal_payload(&tags[check_index], &readers[inrange_readers[j].index]);
                    }else{
                        tag_send_collided_payload(&tags[check_index], &readers[inrange_readers[j].index]);
                    }
                }
                tags[check_index].output.type = DEFAULT_FRAME_TYPE;
                tags[check_index].state = UPLINK_SEND_START;
                tags[check_index].check_time = TIME_INF;
            }
            break;

        default:
            break;
    }
}

void get_next_checkpoint(int *check_index, int *check_state){
    uint64_t min_time = TIME_INF;
    for(int i = 0; i < tags.size(); i++){
        if(min_time > tags[i].check_time){
            min_time = tags[i].check_time;
            *check_state = tags[i].state;
            *check_index = i;
        }
    }

    for(int i = 0; i < readers.size(); i++){
        if(min_time > readers[i].check_time){
            min_time = readers[i].check_time;
            *check_state = readers[i].state;
            *check_index = i;
        }
    }
}

FILE *open_file(char *file_name){
    time_t time_now;
    struct tm *curr_time = NULL;
    time(&time_now);
    curr_time = localtime(&time_now);

//    char file_path[256] = {0};
//    sprintf(file_path, "eval_data/res_%02d_%02d_%02d_%02d_%02d.txt", (1 + curr_time->tm_mon),curr_time->tm_mday,curr_time->tm_hour,curr_time->tm_min,curr_time->tm_sec);
    return fopen(file_name, "w+");              
}

case_t test_case[] = {
    //{bitrate, distance, lane, velocity, collocated}
    //working range
#if 0
    {500, UPLINK_MAX_DISTANCE_TYPE2, 1, 1, 0},
#else    
#if TEST_DEF==1
    {125, UPLINK_MAX_DISTANCE_TYPE3_B125, 2, 1 + CUSTOM_OFFSET_INDEX, 0},
#elif TEST_DEF==2    
    {250, UPLINK_MAX_DISTANCE_TYPE3_B250, 2, 1 + CUSTOM_OFFSET_INDEX, 0},
#elif TEST_DEF==3    
    {500, UPLINK_MAX_DISTANCE_TYPE3_B500, 2, 1 + CUSTOM_OFFSET_INDEX, 0},
#elif TEST_DEF==4    
    //{1000, UPLINK_MAX_DISTANCE_TYPE2, 2, 1, 0},
    {1000, UPLINK_MAX_DISTANCE_TYPE3, 2, 1 + CUSTOM_OFFSET_INDEX, 0},
#elif TEST_DEF==5    
    //road traffic pattern
    {1000, UPLINK_MAX_DISTANCE_TYPE3, 1, 0 + CUSTOM_OFFSET_INDEX, 0},
#elif TEST_DEF==6    
//    {1000, UPLINK_MAX_DISTANCE_TYPE3, 2, 1, 0},
    {1000, UPLINK_MAX_DISTANCE_TYPE3, 3, 2 + CUSTOM_OFFSET_INDEX, 0},
#elif TEST_DEF==7    
    //RetroSign Density
    //{1000, UPLINK_MAX_DISTANCE_TYPE3, 2, 1, 0},
    {1000, UPLINK_MAX_DISTANCE_TYPE3, 2, 1 + CUSTOM_OFFSET_INDEX, 1},
#else
    {1000, UPLINK_MAX_DISTANCE_TYPE3, 2, 1 + CUSTOM_OFFSET_INDEX, 2},    
#endif    
#endif    
    {0,0,0,0,0}
};

int spacing_tbl[] = {25, 50, 100, 200, 400, 800, 0};
//int spacing_tbl[] = {800, 0};
int main(){
//    srand(2);
    for(int test_round = 0; test_round < 100; test_round++){
    srand(time(NULL));
    char fold_name[128] = {};
    time_t time_now;
    struct tm *curr_time = NULL;
    time(&time_now);
    curr_time = localtime(&time_now);
    sprintf(fold_name, "eval_data/res_%02d_%02d_%02d_%02d_%02d", (1 + curr_time->tm_mon),curr_time->tm_mday,curr_time->tm_hour,curr_time->tm_min,curr_time->tm_sec);
    if(access(fold_name, 0) != 0){
        mkdir(fold_name, 0777);
    }

    for(int case_index = 0; test_case[case_index].uplink_bitrate; case_index++){
        for(int spacing_index = 0; spacing_tbl[spacing_index]; spacing_index++){
            generate_readers(60, test_case[case_index]); //1000
            generate_tags(80, test_case[case_index], spacing_tbl[spacing_index]);  //100

            char file_name[256] = {};
            memset(file_name, 0, sizeof(file_name));
            sprintf(file_name, "%s/bitrate%d_uplinkdistance%d_lane%d_velocity%d_collocated%d_spacing%d_downlink%d_readerdensity%f.txt", fold_name,
            test_case[case_index].uplink_bitrate, test_case[case_index].uplink_max_distance, test_case[case_index].lane_num, test_case[case_index].velocity_type, test_case[case_index].collocated_tag_num, spacing_tbl[spacing_index], DOWNLINK_BITRATE, test_case[case_index].lane_num*100.0/velocity_dist_tbl[test_case[case_index].velocity_type][1]);
            

            test_counter = 0;

            while(1){
                int check_index = 0;
                int check_state = 0;
                get_next_checkpoint(&check_index, &check_state);
                update_device(check_index, check_state);
                if(readers[0].init_posx + readers[0].velocity * readers[0].check_time > tags[tags.size()-1].init_posx){
                    FILE *f = open_file(file_name);
                    float sum = 0;
                    for(int i = 0; i < readers.size(); i++){
                        LOG(DEBUG, "readers[%d].tagID.size():%ld, (%.8f, %.8f)", i, readers[i].tagID.size(), readers[i].init_posx, readers[i].init_posy);
                        char write_str[256];
                        sprintf(write_str, "%f %f %ld %d %d %f %d %d %f %ld\n", 
                        readers[i].init_posx, 
                        readers[i].init_posy, 
                        readers[i].tagID.size(),
                        downlink_send_success_count,
                        downlink_send_success_count+downlink_send_fail_count,
                        (float)downlink_send_success_count/(downlink_send_success_count+downlink_send_fail_count),
                        uplink_send_success_count[i],
                        uplink_send_success_count[i]+uplink_send_fail_count[i],
                        (float)uplink_send_success_count[i]/(uplink_send_success_count[i]+uplink_send_fail_count[i]),
                        tags.size()
                        );
                        uplink_send_success_count[i] = 0;
                        uplink_send_fail_count[i] = 0;
                        sum += readers[i].tagID.size();
                        fwrite(write_str, sizeof(char), strlen(write_str), f);                
                    }

                    LOG(INFO, "downlink successful rate: %f, downlink_send_success_count:%d, total:%d",
                    (float)downlink_send_success_count/(downlink_send_success_count+downlink_send_fail_count),
                    downlink_send_success_count,
                    downlink_send_success_count+downlink_send_fail_count);
                    downlink_send_success_count = 0;
                    downlink_send_fail_count = 0;

#if 0
                    LOG(ERR, "uplink successful rate: %f, uplink_send_success_count:%d, total:%d",
                    (float)uplink_send_success_count/(uplink_send_success_count+uplink_send_fail_count),
                    uplink_send_success_count,
                    uplink_send_success_count+uplink_send_fail_count);
                    uplink_send_success_count = 0;
                    uplink_send_fail_count = 0;
#endif


                    LOG(INFO, "%s result:%.8f", file_name, sum/readers.size());
                    fclose(f);
                    break;
                }
            }
            LOG(INFO, "test counter:%ld", test_counter);
            readers.clear();
            tags.clear();
        }
    }
    }
}
