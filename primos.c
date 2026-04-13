#define _POSIX_C_SOURCE 199309L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#define TIMEOUT_SEC 59.2
#define SEG_BITS (1u << 22)
#define SEG_BYTES (SEG_BITS >> 3)
#define OBUF_SIZE (1u << 26)

static double t0;

static inline double wtime(){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return ts.tv_sec + ts.tv_nsec*1e-9;
}

static char dp[200];

static void init_dp(){
    for(int i=0;i<100;i++){
        dp[2*i]='0'+i/10;
        dp[2*i+1]='0'+i%10;
    }
}

static inline int u64tostr(char *out,uint64_t v){
    char tmp[20];
    int n=0;

    while(v>=100){
        uint64_t q=v/100;
        int r=(int)(v-q*100)<<1;
        tmp[n++]=dp[r+1];
        tmp[n++]=dp[r];
        v=q;
    }

    if(v>=10){
        int r=(int)v<<1;
        tmp[n++]=dp[r+1];
        tmp[n++]=dp[r];
    } else tmp[n++]='0'+(int)v;

    for(int i=0;i<n;i++) out[i]=tmp[n-1-i];
    return n;
}

static uint32_t *bp;
static uint32_t bpcnt;

static void make_base(){
    const uint32_t sq=320000;
    uint8_t *s=malloc(sq+1);
    memset(s,1,sq+1);
    s[0]=s[1]=0;

    for(uint32_t i=2;(uint64_t)i*i<=sq;i++)
        if(s[i])
            for(uint32_t j=i*i;j<=sq;j+=i)
                s[j]=0;

    bpcnt=0;
    for(uint32_t i=2;i<=sq;i++)
        if(s[i]) bpcnt++;

    bp=malloc(bpcnt*sizeof(*bp));

    int k=0;
    for(uint32_t i=2;i<=sq;i++)
        if(s[i]) bp[k++]=i;

    free(s);
}

typedef struct{
    uint64_t lo_odd;
    uint8_t sv[SEG_BYTES];
} Seg;

static void sieve_seg(Seg *g){
    uint64_t lo=g->lo_odd;
    uint64_t hi=lo+2*(uint64_t)(SEG_BITS-1);

    memset(g->sv,0xFF,SEG_BYTES);

    if(lo==1) g->sv[0]&=~1u;

    for(uint32_t i=1;i<bpcnt;i++){
        uint64_t p=bp[i];
        if(p*p>hi) break;

        uint64_t start;

        if(p*p>=lo) start=p*p;
        else{
            start=((lo+p-1)/p)*p;
            if(!(start&1)) start+=p;
        }

        uint64_t base=(start-lo)>>1;

        for(uint64_t j=base;j<SEG_BITS;j+=p)
            g->sv[j>>3]&=(uint8_t)(~(1u<<(j&7)));
    }
}

typedef struct{
    pthread_t tid;
    pthread_mutex_t mu;
    pthread_cond_t ci,co;
    Seg *job;
    int go,done,quit;
} TW;

static Seg segs[2];
static TW tw[2];

static void *worker(void *a){
    TW *w=a;

    for(;;){
        pthread_mutex_lock(&w->mu);

        while(!w->go && !w->quit)
            pthread_cond_wait(&w->ci,&w->mu);

        if(w->quit){
            pthread_mutex_unlock(&w->mu);
            return NULL;
        }

        w->go=0;
        pthread_mutex_unlock(&w->mu);

        sieve_seg(w->job);

        pthread_mutex_lock(&w->mu);
        w->done=1;
        pthread_cond_signal(&w->co);
        pthread_mutex_unlock(&w->mu);
    }
}

static char obuf[OBUF_SIZE];
static size_t opos=0;
static int first=1;
static uint64_t total=0;

static void flush(){
    fwrite(obuf,1,opos,stdout);
    opos=0;
}

static void emit(Seg *g){
    uint64_t lo=g->lo_odd;
    const uint64_t *sv=(const uint64_t*)g->sv;
    uint32_t nw=SEG_BYTES/8;

    for(uint32_t k=0;k<nw;k++){
        if(wtime()-t0>=TIMEOUT_SEC) return;

        uint64_t w=sv[k];
        if(!w) continue;

        uint64_t base=lo+((uint64_t)k<<7);

        while(w){
            int b=__builtin_ctzll(w);
            w&=w-1;

            uint64_t p=base+(uint64_t)b*2;

            if(opos+32>=OBUF_SIZE) flush();

            if(!first){
                obuf[opos++]=',';
                obuf[opos++]=' ';
            }

            first=0;
            opos+=u64tostr(obuf+opos,p);
            total++;
        }
    }
}

int main(){
    t0=wtime();
    init_dp();

    int n=sysconf(_SC_NPROCESSORS_ONLN);
    if(n<1) n=1;
    if(n>2) n=2;

    make_base();

    for(int i=0;i<n;i++){
        pthread_mutex_init(&tw[i].mu,NULL);
        pthread_cond_init(&tw[i].ci,NULL);
        pthread_cond_init(&tw[i].co,NULL);
        tw[i].go=tw[i].done=tw[i].quit=0;
        pthread_create(&tw[i].tid,NULL,worker,&tw[i]);
    }

    obuf[0]='2';
    opos=1;
    first=0;
    total=1;

    uint64_t cur=1;

    while(1){
        double now=wtime();

        if(now-t0>=TIMEOUT_SEC) break;
        if(TIMEOUT_SEC-(now-t0)<0.12) break;

        int jobs=0;

        for(int i=0;i<n;i++){
            if(wtime()-t0>=TIMEOUT_SEC) break;

            segs[i].lo_odd=cur;
            tw[i].job=&segs[i];

            pthread_mutex_lock(&tw[i].mu);
            tw[i].done=0;
            tw[i].go=1;
            pthread_cond_signal(&tw[i].ci);
            pthread_mutex_unlock(&tw[i].mu);

            cur+=2*SEG_BITS;
            jobs++;
        }

        for(int i=0;i<jobs;i++){
            pthread_mutex_lock(&tw[i].mu);
            while(!tw[i].done)
                pthread_cond_wait(&tw[i].co,&tw[i].mu);
            pthread_mutex_unlock(&tw[i].mu);

            emit(&segs[i]);
        }
    }

    flush();
    fflush(stdout);

    printf("\nQuantidade: %lu\n",total);
    printf("Tempo: %.2fs\n",wtime()-t0);

    for(int i=0;i<n;i++){
        pthread_mutex_lock(&tw[i].mu);
        tw[i].quit=1;
        pthread_cond_signal(&tw[i].ci);
        pthread_mutex_unlock(&tw[i].mu);
        pthread_join(tw[i].tid,NULL);
    }

    free(bp);
    return 0;
}