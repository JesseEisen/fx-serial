#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <assert.h>
#include <signal.h>
#include <pthread.h>
#include <stdarg.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include "fx-serial.h"

#define MTU 4096
//////////////////////////////////////////////////////////////////
// DEBUG lib
// use DEBUG_PRINT to open/close debug info
//////////////////////////////////////////////////////////////////
pthread_mutex_t DEBUG_LOCK = PTHREAD_MUTEX_INITIALIZER;

#if defined(DEBUG_PRINT)
#define DEBUG(...) \
	do { \
		pthread_mutex_lock(&DEBUG_LOCK);\
		fprintf(stderr, "[DEBUG]:%s:%d:%s(): ", __FILE__, __LINE__, __FUNCTION__); \
		fprintf(stderr, __VA_ARGS__); \
		pthread_mutex_unlock(&DEBUG_LOCK);\
	} while(0)
#else
void no_op(){}
#define DEBUG(...) no_op()
#endif /* NO_DEBUG */

//////////////////////////////////////////////////////////////////

// priority queue 
// for inter thread comminication
//////////////////////////////////////////////////////////////////
#define PRI_MAX 10
#define BUF_POOL_SIZE 65536
#define LOG
#define LOG2
#define LOG3

#define ASSERT(x) assert(x)
#define uint32_t int
#define LOCK(x) pthread_mutex_lock(&x)
#define UNLOCK(x) pthread_mutex_unlock(&x)

typedef enum bool_ {
	false,
	true
} bool;

typedef struct queue_stats_ {
	int enqueue;
	int dequeue;
	int wait_time;
} queue_stats;

int priority[PRI_MAX];

/*
 * List of nodes in a hash bucket
 */
typedef struct node_ {
	void* key;
	int priority;
	struct node_* next;
} node;

/*
 * Define the hash table
 * |p1| ->|a|->|b|->|c|
 * |p2|->|e|->|f|
 */
typedef struct ptable_entry_ {
	int priority;
	node* n;
} ptable_entry;

typedef struct ptable_ {
	ptable_entry entry[PRI_MAX];
	node* last[PRI_MAX];
	node* buf_pool;
	node* free_bbuf_pool;
	int ent_count;
	pthread_mutex_t lock;
	pthread_cond_t cv;
	bool is_available;
	queue_stats *stats;
} ptable;

void add_a_node(ptable* p, node** last, node** m, void* key, int priority);
void* get_buf(ptable *p);
void put_buf(ptable* p, void* buf);
void display_buf_pool(ptable* p);
void create_pool(ptable** p, uint32_t num);
void create(ptable* p);
void put_data(ptable* p, void* key, int priority);
void* get_data(ptable* p, int* pri);
void cleanup(ptable *p);
void display(ptable* p);

/*
 * Adds a node of a given priority to the queue. Since a node is
 * allocated from a fixed size buffer pool, this function blocks
 * if pool has no free buffer object. 
 */
void add_a_node(ptable* p, node** last, node** m, void* key, int priority)
{
	ASSERT(p);

	LOCK(p->lock);
	node *n = NULL;

	n = (node*)get_buf(p);

	LOG3("oo-get_data-oo\n");
	display_buf_pool(p);
	LOG3("---get_data--\n");

	if (NULL == n) {
		LOG2("Buf pool is over. Waiting for dequeue\n");
		pthread_cond_wait(&p->cv, &p->lock);
		n = (node*)get_buf(p);
		LOG2("Producer: wait over. Got a buffer back\n");
	}
	/*
	 * Collided nodes are arranged in a list (queue)
	 */
	n->key = key;
	n->priority = priority;
	n->next = NULL;

	if (NULL == *m) {
		*m = n;
	} else {
		(*last)->next = n;
	}

	*last = n;
	LOG("Enqueue: %d\n", p->stats->enqueue++);

	p->is_available = true;
	pthread_cond_signal(&p->cv);
	UNLOCK(p->lock);
}

/*
 * Gets a buffer from the buffer pool
 */
void* get_buf(ptable *p)
{
	/*
	 * Check if we have at least two nodes
	 */
	node* head = p->buf_pool;

	if(p->buf_pool != NULL) {
		p->buf_pool = head->next;
		LOG2("Stealing a buffer %p\n", head);
		return head;
	} else {
		LOG2("\nBuffer overrun\n");
		return NULL;
	}
}

/*
 * Returns a buffer to buffer pool
 */
void put_buf(ptable* p, void* buf)
{
	if (p->buf_pool) {
		node* head = (node*)buf;
		head->next = p->buf_pool;
		p->buf_pool = head;
		LOG2("Unstealing a buffer %p\n", buf);
	} else {
		p->buf_pool = buf;
		LOG2("Unstealing the last buffer %p\n", buf);
	}
}

void display_buf_pool(ptable* p)
{
	ASSERT(p);

	int i = 1;
	node* temp = p->buf_pool;

	while(temp) {
		LOG2("Buf %d: %p\n", i++, temp);
		temp = temp->next;
	}
}

void create_pool(ptable** p, uint32_t num)
{
	node* head= NULL;
	node* temp = NULL;

	int i = 0;

	head = malloc(sizeof(node)); 
	memset(head, 0, sizeof(node));
	assert(head);

	temp = head;

	for(i = 1; i < num; i++) {
		temp->next = malloc(sizeof(node));
		memset(temp->next, 0, sizeof(node));
		assert(temp->next);
		temp = temp->next;
	}
	temp->next = NULL;

	/*
	 * Set the buf pool
	 */
	if (NULL == (*p)->buf_pool) {
		(*p)->buf_pool = head;
	}

#ifdef DEBUG
	display_buf_pool(*p);
#endif

}

/*
 * Create a priority queue object of priority ranging from 0..PRIMAX-1
 */
void create(ptable* p)
{
	ASSERT(p);

	int i = 0;

	/*
	 * Initialize the entries
	 */
	for(i = 0; i < PRI_MAX; i++) {
		p->entry[i].priority = i;
		p->entry[i].n = NULL;
		p->last[i] = NULL;
	}

	create_pool(&p, BUF_POOL_SIZE);

	p->stats = malloc(sizeof(queue_stats));
	memset(p->stats, 0, sizeof(node));
	assert(p->stats);

	memset ( &(p->lock), 0, sizeof(pthread_mutex_t));
	memset ( &(p->cv), 0, sizeof(pthread_cond_t));
	pthread_mutex_init(&(p->lock), NULL);
	pthread_cond_init(&(p->cv), NULL);

	p->is_available = false;
	p->ent_count = PRI_MAX;
}

/*
 * Adds a node to the queue 
 */
void put_data(ptable* p, void* key, int priority)
{
	ASSERT(p);
	ASSERT(priority < PRI_MAX);

	add_a_node(p, &(p->last[priority]), &(p->entry[priority].n),
			key, priority);
}

/*
 * Gets the highest priority node from the queue. If queue is empty,
 * then this routine blocks.
 */
void* get_data(ptable* p, int* pri)
{
	ASSERT(p);

	LOCK(p->lock);
	int i = 0;
	node* temp = NULL;
	void *key;

wait_again:
	while (false == p->is_available) {
		/*
		 * Else wait for the next element to get in
		 */
		LOG2("Nothing in queue; waiting for data\n");
		pthread_cond_wait(&p->cv, &p->lock);
		LOG2("Waiting completed: got data\n");
	}

	for (i = 0; i < PRI_MAX; i++) {
		if (NULL != p->entry[i].n) {
			temp = (p->entry[i].n);

			key = p->entry[i].n->key;
			if (pri) *pri = p->entry[i].n->priority;

			p->entry[i].n = temp->next;

			LOG(" Dequeued: %d\n", p->stats->dequeue++);
			put_buf(p, temp);
#ifdef DEBUG
			LOG3("oo-put_data-oo\n");
			display_buf_pool(p);
			LOG3("---put_data--\n");
#endif
			pthread_cond_signal(&p->cv);
			UNLOCK(p->lock);
			return key;
		}
	}
	p->is_available = false;
	goto wait_again;
}

void cleanup(ptable *p)
{
	node *n = p->buf_pool;

	while(n) {
		node* temp = n;
		n = n->next;
		free(temp);
	}
	free(p->stats);

	pthread_mutex_destroy(&(p->lock));
	pthread_cond_destroy(&(p->cv));
	free(p);
}

/*
 * Function to display the queue
 */
void display(ptable* p)
{
	ASSERT(p);
	int i = 0;
	node* t = NULL;
	for(i = 0; i < PRI_MAX; i++) {
		t = p->entry[i].n;
		while(t) {
			printf("\nBucket=%d|Key=%d|Priority=%d\n", p->entry[i].priority,
					*((int *)(t->key)),
					t->priority);
			t = t->next;
		}
	}
}
// End priority queue
//////////////////////////////////////////////////////////////////

// Helper Functions
//////////////////////////////////////////////////////////////////
static int safe_read(int fd, void *buffer, int n)
{
	fd_set fds;
	int cnt;

	if (fd < 0) {
		return -1;
	}

	for (;;) {
		FD_ZERO(&fds);
		FD_SET(fd, &fds);

		cnt = select(fd+1, &fds, NULL, NULL, NULL);
		if (cnt < 0) {
			return -1;
		}

		//		int size;
		//		int ret = ioctl(fd, FIONREAD, &size);
		//		if (ret < 0) return -1;
		//		if (size == 0) continue;

		cnt = read(fd, buffer, n);
		if (cnt != 0) {
			return cnt;
		}

		if (cnt == 0) return -1;
	}
}

static int safe_write(int fd, void *buffer, int count)
{
	int actual = 0;

	if (fd < 0) return -1;

	while (count > 0) {
		int n = write(fd, buffer, count);
		if (n < 0 && errno == EINTR) {
			continue;
		}

		if (n <= 0) {
			actual = -1;
			break;
		}

		count  -= n;
		actual += n;
		buffer += n;
	}

	return actual;
}
//////////////////////////////////////////////////////////////////

// serial operation
//////////////////////////////////////////////////////////////////
typedef int (*serial_cb)(int fd, char *msg, int sz);

struct serialcommand {
	int fd;
	serial_cb cb;
	int sz;
	char buf[4096];
};

struct fx_serial {
	char device[255];
	struct {
		int baude;
		char bits;
		char parity;
		char stop;
	} config;
	
	int fd;

	struct {
		int n_send;
		int n_recv;
		int n_err;
	} stats;

	ptable *req; // queue
	pthread_t tid_serial;
};

static int _open_device(struct fx_serial *s, char *device)
{
	assert(s);
	assert(device);
	assert(strlen(device) < 255);

	memset(s, 0, sizeof(*s));
	strcpy(s->device, device);

	s->fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (s->fd == -1) {
		DEBUG("fd = %d, %s\n", s->fd, strerror(errno));
		return -1;
	}

	s->req = malloc(sizeof(ptable));
	create(s->req);
	
	return 0;
}

static int _set_device(struct fx_serial *s, int baude, char bits, char parity, char stop)
{
	int fd = s->fd;

	struct termios options;
	if (tcgetattr(fd, &options) != 0) {
		return -1;
	}

	bzero(&options, sizeof(struct termios));

	switch (baude) {
	case 9600:
		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);
		break;
	default:
		return -1;
	}

	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	/*Input */
	options.c_oflag &= ~OPOST;	/*Output */

	options.c_cflag |= CLOCAL;
	options.c_cflag |= CREAD;

	switch (parity) {
	case 'N':
	case 'n':
	case 'S':
	case 's':
		options.c_cflag &= ~PARENB;
		break;
	case 'E':
	case 'e':
		options.c_cflag |= PARENB;
		options.c_cflag &= ~PARODD;
		break;
	case 'O':
	case 'o':
		options.c_cflag |= PARENB;
		options.c_cflag |= PARODD;
		break;
	default:
		return -1;
	}

	if (options.c_cflag & PARENB) {
		// Enable Parity
		options.c_iflag |= (INPCK | ISTRIP);
	}

	switch (stop) {
	case '1':
		options.c_cflag &= ~CSTOPB;
		break;
	default:
		return -1;
	}


	switch (bits) {
	case '7':
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;
		break;
	case '8':
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
		break;
	default:
		return -1;
	}

	options.c_cc[VTIME] = 0;	/* 设置超时15 seconds */
	options.c_cc[VMIN] = 1;	/* define the minimum bytes data to be readed */
	tcflush(fd, TCIFLUSH);

	// set Attr
	if (tcsetattr(fd, TCSANOW, &options) != 0) {
		return -1;
	}

	return 0;
}

static int _close_device(struct fx_serial *s)
{
	assert(s);
	
	if (s->fd > 0) close(s->fd);
	if (s->req) cleanup(s->req);

	memset(s, 0, sizeof(struct fx_serial));
	
	return 0;
}


static int _check_command(char *buf, int sz)
{
	// TODO: implementation totally check

	int ret = 0;
	
	ret |= (buf[0] == 0x02);
	ret |= (buf[1] == 0x30 || buf[1] == 0x31);

	return ret;
}

void *thread_serialcomm(void *parm)
{
	struct fx_serial *s = (struct fx_serial*)parm;
	
RESTART:
	while (1) {
		usleep(1000);
		struct serialcommand *sc = get_data(s->req, NULL);
		int ret;

		// clean serial
//		struct timeval tv_clean;
//		memset(&tv_clean, 0, sizeof(struct timeval));
//		tv_clean.tv_sec  = 0;
//		tv_clean.tv_usec = 0;	
//
//		fd_set rfds_clean;
//		FD_ZERO(&rfds_clean);
//		FD_SET(s->fd, &rfds_clean);
//		
//		int ret = select(s->fd+1, &rfds_clean, NULL, NULL, &tv_clean);	
//		if (ret == -1) {
//			DEBUG("select error\n");
//			free(sc);
//			goto RESTART;
//		} else if (ret == 0) {
//			DEBUG("never happen\n");
//			free(sc);
//			goto RESTART;
//		} else {
//			char tmp[4096];
//			read(s->fd, tmp, 4096);
//			DEBUG("clean plc serial buffer\n");
//		}
			
		if (_check_command(sc->buf, sc->sz) == 0) {
			DEBUG("cmd error\n");
			free(sc);
			goto RESTART;
		}

		int num = 0;
		if (sc->buf[1] == 0x30) {
			// DATA size + STX(1 byte) + ETX(1 byte) + SUM(2 byte)
			num = ((sc->buf[6]-'0')*10 + (sc->buf[7]-'0'))*2+4;
		} else {
			num = 1;
		}

		if (num > 64*2+4) {
			free(sc);
			goto RESTART;
		}


		// write command
		ret = safe_write(s->fd, sc->buf, sc->sz);
		if (ret < 0) {
			free(sc);
			goto RESTART;
		}

		s->stats.n_send++;
		
		char resp[4096];
		memset(resp, 0, sizeof(resp));
		int sz = 0;
		char *p_resp = resp;

		for (;;) {
			struct timeval tv;
			memset(&tv, 0, sizeof(struct timeval));
			tv.tv_sec = 5;
			tv.tv_usec = 0;	

			fd_set rfds;
			FD_ZERO(&rfds);
			FD_SET(s->fd, &rfds);
	
			ret = select(s->fd+1, &rfds, NULL, NULL, &tv);
			if (ret == -1) {
				DEBUG("select error\n");
				free(sc);
				goto RESTART;
			} else if (ret == 0) {
				DEBUG("time expired\n");
				free(sc);
				goto RESTART;
			} else {
				int cnt = read(s->fd, p_resp, num);
				if (cnt == 0) {
					DEBUG("serial error\n");
					free(sc);
					goto RESTART;
				}


				num -= cnt;
				p_resp += cnt;
				sz += cnt;
				if (num == 0) {
					// call cb
					s->stats.n_recv++;
					sc->cb(sc->fd, resp, sz);
					break;
				}
			}
		}
	
		// free something
		free(sc);
	}

	return (void *)NULL;
}

struct fx_serial* fx_serial_start()
{
	struct fx_serial *s = malloc(sizeof(struct fx_serial));
	assert(s);
	int ret;
	ret = _open_device(s, "/dev/ttyUSB0");
	assert(ret == 0);
	
	ret = _set_device(s, 9600, '7', 'E', '1');
	assert(ret == 0);
	
	pthread_t tid_serial;
	ret = pthread_create(&tid_serial, NULL, thread_serialcomm, (void *)s);
	assert(ret == 0);

	s->tid_serial = tid_serial;
	
	return s;
}

int serial_stop(struct fx_serial *s)
{
	pthread_cancel(s->tid_serial);
	pthread_join(s->tid_serial, NULL);
	_close_device(s);

	return 0;
}

static int serial_command(struct fx_serial *s, struct serialcommand *sc)
{
	assert(s);
	assert(sc);

	struct serialcommand *local_sc = malloc(sizeof(struct serialcommand));
	assert(sc);	
	
	local_sc->fd = sc->fd;
	local_sc->cb = sc->cb;
	
	local_sc->sz = sc->sz;
	memcpy(local_sc->buf, sc->buf, sizeof(local_sc->buf));

	// TODO: let write command has high priority
	put_data(s->req, (void *)local_sc, 1/*Priority*/);
	return 0;
}


static char _getAscii(int i) 
{
	if (i >=0 && i <= 9) return i+'0';
	else if (i>=10 && i <=15) return (i-10)+'A';
}

static char _getAddressAscii(int address, char buf[4])
{
	int x = address * 2 + 0x1000;

	int i, j, m, n;  
	i = x / (16 * 16 * 16); 
	j = (x / (16 * 16)) % 16; 
	m = x - i*(16*16*16) - j*(16*16); 
	m = m / 16; 
	n = x % 16; 

	buf[0] = _getAscii(i);
	buf[1] = _getAscii(j);
	buf[2] = _getAscii(m);
	buf[3] = _getAscii(n);

	//printf("add2ascii:%d->%02x, %02x, %02x, %02x\n", address, buf[0], buf[1], buf[2], buf[3]);
}

static int getReadCommandFrame (char *buf, int *sz, int address, int num)
{
	if (buf == NULL || sz == NULL ||  
			(address < 0 || address > 255) || num < 0)  
		return -1; 

	buf[0] = 0x02;
	buf[1] = '0';

	_getAddressAscii(address, &buf[2]);

	num = num *2;

	buf[6] = _getAscii(num/10);
	buf[7] = _getAscii(num%10);

	buf[8] = 0x03;


	int i, sum = 0;
	for (i = 1; i <= 8; i++)
		sum += buf[i];

	i = sum&0xFF;
	buf[9]  = _getAscii(i/16);
	buf[10] = _getAscii(i%16);

	*sz = 11;

	for (i = 0; i < *sz; i++) {
//		printf("%02x ", buf[i]);
	}

//	printf("\n");

	return 0;
}

static int getWriteCommandFrame(char *buf, int *sz, int address, int num, char *data)
{
	if (buf == NULL || sz == NULL ||  
			(address < 0 || address > 255) || num < 0)  
		return -1; 

	buf[0] = 0x02;
	buf[1] = '1';

	_getAddressAscii(address, &buf[2]);

	num = num*2;

	buf[6] = _getAscii(num/10);
	buf[7] = _getAscii(num%10);

	int i;
	for (i = 0; i < num*2; i+=4) {
		buf[8+i]   = data[i+2];
		buf[8+i+1] = data[i+3];
		buf[8+i+2] = data[i];
		buf[8+i+3] = data[i+1];
	}

	buf[8+num*2] = 0x03;

	int sum = 0;
	for (i = 1; i <= 8+num*2; i++)
		sum += buf[i];

	i = sum&0xFF;
	buf[8+num*2+1]  = _getAscii(i/16);
	buf[8+num*2+2]  = _getAscii(i%16);

	*sz = 8+num*2+3;

	for (i = 0; i < *sz; i++) {
//		printf("%02x ", buf[i]);
	}

//	printf("\n");

	return 0;
}
//////////////////////////////////////////////////////////////////

static int _cb_async(int fd, char *buf, int sz)
{	
	write(fd, buf, sz);
	return 0;
}

int controller_set(struct fx_serial *s, int id, int status)
{
	struct serialcommand sc;
	
	switch(status) {
	case 0:
		getWriteCommandFrame(sc.buf, &sc.sz, id, 1, "0000");
		break;
	case 1:
		getWriteCommandFrame(sc.buf, &sc.sz, id, 1, "0001");
		break;
	default:
		fprintf(stderr, "device status number error\n");
		return -1;
	}	

	int fd[2];
	pipe(fd);

	sc.fd = fd[1];	
	sc.cb = _cb_async;
	serial_command(s, &sc);
	
	char buf[255];
	int sz;
	int ret; /*select return value*/
	struct timeval tv;
	fd_set readset;

	FD_ZERO(&readset);
	FD_SET(fd[0],&readset);
	tv.tv_sec = 2;
	tv.tv_usec = 0;
	
	if((ret = select((fd[0]+1),&readset,NULL,NULL,&tv)) < 0){
		printf("select error\n");
		return -1;
	}else if(ret == 0){
		printf("filedes not ready\n");
		return -1;
	} 

	sz = read(fd[0], buf, 255);
	int i;
	for (i = 0; i < sz; i++) {
		printf("%02x ", buf[i]);
	}

	printf("\n");
	close(fd[0]);
	close(fd[1]);

	if (buf[0] == 0x06) {
		return 0;
	} else {
		return -1;
	}	
} 

int controller_get(struct fx_serial *s, int id, int *status)
{
	struct serialcommand sc;
	getReadCommandFrame(sc.buf, &sc.sz, id, 1);

	int fd[2];
	pipe(fd);

	sc.fd = fd[1];	
	sc.cb = _cb_async;
	serial_command(s, &sc);
	
	char buf[255];
	int  sz;
	int  ret; /*select return*/
	struct timeval tv;
	fd_set readset;
	
	tv.tv_sec = 2;
	tv.tv_usec = 0;
	FD_ZERO(&readset);
	FD_SET(fd[0],&readset);/*let fd[0] seted in readset*/
	if((ret = select(( fd[0]+1),&readset,NULL,NULL,&tv))< 0){
		printf("select error\n");
		return -1;
	}else if(ret == 0){
		printf("filedes are not ready for read within 5s\n");
		return -1;
	}
	sz = read(fd[0], buf, 255);
	
	int i;
	for (i = 0; i <  sz; i++) {
		printf("%02x ", buf[i]);
	}

	printf("\n");
	close(fd[0]);
	close(fd[1]);

//	printf("buf[2] = %x\n", buf[2]);
	if (buf[2] == 0x31) {
		*status = 1;
	} else if (buf[2] == 0x30) {
		*status = 0;
	} else {
		return -1;
	}

	return 0;
}
static int  atoh(char x)
{
	int y;
	switch(x)
	{
		case '1': y = 1; break;
		case '2': y = 2; break;
		case '3': y = 3; break;
		case '4': y = 4; break;
		case '5': y = 5; break;
		case '6': y = 6; break;
		case '7': y = 7; break;
		case '8': y = 8; break;
		case '9': y = 9; break;
		case 'A': y = 10; break;
		case 'B': y = 11; break;
		case 'C': y = 12; break;
		case 'D': y = 13; break;
		case 'E': y = 14; break;
		case 'F': y = 15; break;
	}

	return  y;
}

static void buf4_to_integer(char *buf, int *x)
{
	int x1, x2, x3, x4;
	x1 = atoh(buf[2]);
	x2 = atoh(buf[3]);
	x3 = atoh(buf[0]);
	x4 = atoh(buf[1]);

	*x = x1*16*16*16+x2*16*16+x3*16+x4;
}

static void integer_to_buf4(int x, char *buf)
{
	int x1, x2, x3, x4;
	x4 = x%10;
	x3 = (x%100)/10;
	x2 = (x/100)%10;
	x1 = x/1000;

	buf[0] = x1 + '0';
	buf[1] = x2 + '0';
	buf[2] = x3 + '0';
	buf[3] = x4 + '0';
}

int sensor_set(struct fx_serial *s, int id, int data)
{
	struct serialcommand sc;
	char buf[4];
	integer_to_buf4(data, buf);
	getWriteCommandFrame(sc.buf, &sc.sz, id, 1, buf);

	int fd[2];
	pipe(fd);

	sc.fd = fd[1];	
	sc.cb = _cb_async;
	serial_command(s, &sc);
	
	char buf2[255];
	int sz;
	int ret; /*select return value*/
	struct timeval tv;
	fd_set readset;

	FD_ZERO(&readset);
	FD_SET(fd[0],&readset);
	tv.tv_sec = 2;
	tv.tv_usec = 0;
	
	if((ret = select((fd[0]+1),&readset,NULL,NULL,&tv)) < 0){
		printf("select error\n");
		return -1;
	}else if(ret == 0){
		printf("filedes not ready\n");
		return -1;
	} 

	sz = read(fd[0], buf2, 255);
	int i;
//	for (i = 0; i < sz; i++) {
//		printf("%02x ", buf2[i]);
//	}

//	printf("\n");
	close(fd[0]);
	close(fd[1]);

	return 0;
}

int sensor_get(struct fx_serial *s, int id, int *data)
{
	struct serialcommand sc;
	getReadCommandFrame(sc.buf, &sc.sz, id, 1);

	int fd[2];
	pipe(fd);

	sc.fd = fd[1];	
	sc.cb = _cb_async;
	serial_command(s, &sc);
	
	char buf[255];
	int sz;
	int ret; /*select return value*/
	struct timeval tv;
	fd_set readset;

	FD_ZERO(&readset);
	FD_SET(fd[0],&readset);
	tv.tv_sec = 2;
	tv.tv_usec = 0;
	
	if((ret = select((fd[0]+1),&readset,NULL,NULL,&tv)) < 0){
		printf("select error\n");
		return -1;
	}else if(ret == 0){
		printf("filedes not ready\n");
		return -1;
	} 

	sz = read(fd[0], buf, 255);
//	int i;
//	for (i = 0; i < sz; i++) {
//		printf("%02x ", buf[i]);
//	}
//
//	printf("\n");
	close(fd[0]);
	close(fd[1]);

	int x;
	buf4_to_integer(&buf[1], &x);
	*data = x;

	return 0;	
}

int sensorX_init(struct sensorX *sx, int n_id, int *id, int *data)
{
	sx->n = n_id;
	int i;
	for (i = 0; i < n_id; i++) {
		sx->id[i] = id[i];
		if (data != NULL) {
			sx->data[i] = data[i];
		}
	}
	return 0;
}

int sensorX_get(struct fx_serial *s, struct sensorX *sx)
{ 
	int i;

	for (i = 0; i < sx->n; i++) {
		if(sensor_get(s, sx->id[i], &sx->data[i]) == -1)
			return -1;			
	}
	return 0;
}

int sensorX_set(struct fx_serial *s, struct sensorX *sx)
{
	int i;
	for (i = 0; i < sx->n; i++) {
		if(sensor_set(s, sx->id[i], sx->data[i]) == -1)
			return -1;
	}
	return 0;
}


