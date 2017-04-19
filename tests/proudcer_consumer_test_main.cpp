#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <iostream>

#include "common/concurrency/producer_consumer_queue.h"
#include "common/concurrency/work_thread.h"
#include "common/concurrency/rwlock.h"
#include<pthread.h>
#include "glog/logging.h"
#include "data_fusion.h"
#include "datafusion_math.h"

struct StructTest
{
//    double timestamp;
//    double att[3];
//    double angle_z;
//    double att_gyro[3];
    int index;
};

using namespace imu;
using namespace folly;

folly::ProducerConsumerQueue<StructTest> queue_list(20);

int g_index = 0;

//void write_att();
//
//void read_att();
//
//void *my_thread_read(void *ptr);
//
//void *my_thread_write(void *ptr);

void *write_att(void *ptr);

void *read_att(void *ptr);

int main(int argc, char *argv[])
{
      // 线程
//    WorkThread m_fusion_thread_write, m_fusion_thread_read;
//    m_fusion_thread_write.Start();
//    Closure<void>* cls_write = NewClosure(this, &write_att);
//    m_fusion_thread.AddTask(cls_write);
//
//    m_fusion_thread_read.Start();
//    Closure<void>* cls_read = NewClosure(this, &read_att);
//    m_fusion_thread_read.AddTask(cls_read);

    pthread_t tid1 ;
    pthread_create(&tid1, NULL, read_att, NULL);

    pthread_t tid ;
    pthread_create(&tid, NULL, write_att, NULL);
    pthread_join(tid,NULL);  // 阻塞主线程，防止程序结束

    return 0;
}


void *write_att(void *ptr)
{
    StructTest att_write;
    while(1){
        att_write.index = g_index++;
        bool write_state = queue_list.write(att_write);
        printf("write write_state = %d, index = %d\n", write_state, att_write.index);
        usleep(10000);
    }
}


void *read_att(void *ptr)
{
    StructTest att_read;
    while(1){
        bool read_state = queue_list.read(att_read);
        printf("--read, read_state = %d, index = %d\n", read_state, att_read.index);
        usleep(100000);
    }
}

