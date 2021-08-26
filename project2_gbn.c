#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ******************************************************************
 ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.1  J.F.Kurose

   This code should be used for Project 2, unidirectional and bidirectional
   data transfer protocols.  Network properties:
   - one way network delay averages five time units (longer if there
     are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
     or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
     (although some can be lost).
**********************************************************************/

#define BIDIRECTIONAL 1

struct event {
   float evtime;           /* event time */
   int evtype;             /* event type code */
   int eventity;           /* entity where event occurs */
   struct pkt *pktptr;     /* ptr to packet (if any) assoc w/ this event */
   struct event *prev;
   struct event *next;
};
struct event *evlist = NULL;   /* the event list */

/* possible events: */
#define  TIMER_INTERRUPT 0
#define  FROM_LAYER5     1
#define  FROM_LAYER3     2

#define  OFF    0
#define  ON     1
#define  A      0
#define  B      1

int TRACE = 1;             /* for my debugging */
int nsim = 0;              /* number of messages from 5 to 4 so far */
int nsimmax = 0;           /* number of msgs to generate, then stop */
float time = 0.000;
float lossprob;            /* probability that a packet is dropped  */
float corruptprob;         /* probability that one bit is packet is flipped */
float lambda;              /* arrival rate of messages from layer 5 */
int   ntolayer3;           /* number sent into layer 3 */
int   nlost;               /* number lost in media */
int   ncorrupt;            /* number corrupted by media*/

/* a packet is the data unit passed from layer 4 (students code) to layer */
/* 3 (teachers code).  Note the pre-defined packet structure, which all   */
/* students must follow. */
struct pkt {
   int seqnum;
   int acknum;
   int checksum;
   int isACK;
   char payload[20];
};

/* a "msg" is the data unit passed from layer 5 (teachers code) to layer  */
/* 4 (students' code).  It contains the data (characters) to be delivered */
/* to layer 5 via the students transport level protocol entities.         */
struct msg {
  char data[20];
};

// Project variables

// Colors
#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define RESET "\x1B[0m"

float   time_ret_pkt_sentA;           // Used to output time of retransmission on success ACKs
float   time_ret_pkt_sentB;
int ret_A;                            // Whether or not this packet was previously retransmitted, used to display time of retransmission
int ret_B;
int total_received_ACKs;              // Total successful ACKs from both sides
struct pkt last_accepted_packet_A;    // The last packet accepted by this side
struct pkt last_accepted_packet_B;
struct pkt last_sent_from_A;          // The last ACK sent by this side
struct pkt last_sent_from_B;

struct pkt sender_buffer_A[50];       // Buffers
struct pkt sender_buffer_B[50];

int base_A;
int base_B;
int next_open_A;
int next_open_B;
int window_A;
int window_B;
int buffer_A;
int buffer_B;
#define WINDOW_SIZE 8

void init();
void generate_next_arrival();
void tolayer5(int AorB, struct msg message);
void tolayer3(int AorB, struct pkt packet);
void starttimer(int AorB, float increment);
void stoptimer(int AorB);
void insertevent(struct event *p);
void init();
float jimsrand();
void printevlist();




/********* STUDENTS WRITE THE NEXT SEVEN ROUTINES *********/
#define TIME_OUT 24.0
#define DEBUG 1

int seq_expect_send_A;	/* Next sequence number to send*/
int seq_expect_recv_A;	/* Next sequence number to receive */
int is_waiting_A;			/* Whether side A is waiting */
int seq_expect_send_B;	/* Next sequence number to send*/
int seq_expect_recv_B;	/* Next sequence number to receive */
int is_waiting_B;			/* Whether side B is waiting */

struct pkt waiting_packet_A;	/* Packet hold in A */
struct pkt waiting_packet_B;	/* Packet hold in A */

/* Print payload */
void print_pkt(char *action, struct pkt packet)
{
	printf("%s: ", action);
	printf("seq = %d, ack = %d, isACK = %d, checksum = %x, ", packet.seqnum, packet.acknum, packet.isACK, packet.checksum);
	int i;
	for (i = 0; i < 20; i++)
		putchar(packet.payload[i]);
	putchar('\n');
}

/* Compute checksum */
int compute_check_sum(struct pkt packet)
{
	int sum = 0, i = 0;
	sum = packet.checksum;
	sum += packet.seqnum;
	sum += packet.acknum;
    sum += packet.isACK;

	sum = (sum >> 16) + (sum & 0xffff);
	for (i = 0; i < 20; i += 2) {
		sum += (packet.payload[i] << 8) + packet.payload[i+1];
		sum = (sum >> 16) + (sum & 0xffff);
	}
	sum = (~sum) & 0xffff;
	return sum;
}

/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message)
{

  if (buffer_A == 50) {
    printf(RED);
    printf("Buffer at full capacity! Dropping packet!\n");
    printf(RESET);
    return;
  }

	/* Send packet to B side */
	memcpy(waiting_packet_A.payload, message.data, sizeof(message.data));
	waiting_packet_A.seqnum = seq_expect_send_A++;
  waiting_packet_A.isACK = 0;
	waiting_packet_A.checksum = 0;
	waiting_packet_A.checksum = compute_check_sum(waiting_packet_A);
	is_waiting_A = 1;
	/* Debug output */
	if (DEBUG)
		print_pkt("Sent from A", waiting_packet_A);

  printf("Buffer at A: filled buffer slots = %d, filled window slots = %d, base A seqnum = %d\n", buffer_A, window_A, sender_buffer_A[base_A % 50].seqnum);
  if (window_A < 8) {
    tolayer3(0, waiting_packet_A);
    if (window_A == 0) {
      starttimer(0, TIME_OUT); // If the current packet being sent is the first/oldest packet in window
    }
    window_A++;
  } else {
    printf("Can't send right now, window is full. Placing in buffer.\n");
  }

  sender_buffer_A[next_open_A] = waiting_packet_A;
  next_open_A = (next_open_A + 1) % 50;
  buffer_A++;
}

void B_output(struct msg message)
{

  if (buffer_B == 50) {
    printf(RED);
    printf("Buffer at full capacity! Dropping packet!\n");
    printf(RESET);
    return;
  }

	/* Send packet to A side */
	memcpy(waiting_packet_B.payload, message.data, sizeof(message.data));
	waiting_packet_B.seqnum = seq_expect_send_B++;
  waiting_packet_B.isACK = 0;
	waiting_packet_B.checksum = 0;
	waiting_packet_B.checksum = compute_check_sum(waiting_packet_B);
	is_waiting_B = 1;
	/* Debug output */
	if (DEBUG)
		print_pkt("Sent from B", waiting_packet_B);

  printf("Buffer at B: filled buffer slots = %d, filled window slots = %d, base A seqnum = %d\n", buffer_B, window_B, sender_buffer_B[base_B % 50].seqnum);
  if (window_A < 8) {
    tolayer3(1, waiting_packet_B);
    if (window_B == 0) {
      starttimer(1, TIME_OUT); // If the current packet being sent is the first/oldest packet in window
    }
    window_B++;
  } else {
    printf("Can't send right now, window is full. Placing in buffer.\n");
  }

  sender_buffer_B[next_open_B] = waiting_packet_B;
  next_open_B = (next_open_B + 1) % 50;
  buffer_B++;
}


/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
    if (DEBUG)
		print_pkt("Received at A", packet);

    int ans_checksum = packet.checksum;
    packet.checksum = 0;
    if (compute_check_sum(packet) != ans_checksum) {
          printf(RED);
          print_pkt("Checksum error at A", packet);
          printf(RESET);
          struct pkt nakpkt;
          nakpkt.acknum = -1;
          nakpkt.isACK = 1;
          nakpkt.checksum = 0;
          nakpkt.checksum = compute_check_sum(nakpkt);
          printf(YEL);
          printf("Sent NAK from A\n");
          printf(RESET);
          //last_sent_from_A = nakpkt;
          tolayer3(0, nakpkt);
      return;
    }
    packet.checksum = ans_checksum;

    if(packet.isACK == 1) {
        if (packet.acknum >= sender_buffer_A[base_A % 50].seqnum) {	/* ACK */
          stoptimer(0);
            if (ret_A == 1) {
              printf(GRN);
              printf("A just received ACK from B for a packet previously retransmitted at time %f\n", time_ret_pkt_sentA);
              printf(RESET);
              ret_A = 0;
            }
            printf(GRN);
            printf("Base A seqnum is %d\n", sender_buffer_A[base_A % 50].seqnum);
            for (int i = sender_buffer_A[base_A % 50].seqnum; i <= packet.acknum; i++) {
              total_received_ACKs++;
              base_A = (base_A + 1) % 50;
              buffer_A--;
              window_A--;
              printf("Total successful ACKs: %d\n", total_received_ACKs);
            }
            if(window_A > 0) {
              starttimer(0, TIME_OUT);
            }
            printf(RESET);
            is_waiting_A = 0;
        } else if(packet.acknum > 0 && packet.acknum < sender_buffer_A[base_A % 50].seqnum) {
          printf(YEL);
          printf("Received ACK %d when base A seqnum is %d. Ignore\n", packet.acknum, sender_buffer_A[base_A % 50].seqnum);
          printf(RESET);
        } else if (packet.acknum == -1) {		/* NAK */

            printf(YEL);
            printf("Received NAK\n");
            if (window_A > 0) {
              stoptimer(0);
              printf("Go back to %d\n", sender_buffer_A[base_A % 50].seqnum);
              for (int i = base_A; i < (base_A + window_A); i++) {
                printf(YEL);
                printf("Retransmitted packet seqnum %d\n", sender_buffer_A[i % 50].seqnum);
                tolayer3(0, sender_buffer_A[i % 50]);
              }
              starttimer(0, TIME_OUT);
            } else {
              printf("Empty window. Resending last sent ACK with acknum %d\n", last_sent_from_A.acknum);
              tolayer3(0, last_sent_from_A);
            }
            printf(RESET);

        }
    }else if (packet.seqnum == seq_expect_recv_A) {
  		/* Pass data to layer5 */
  		struct msg message;
  		memcpy(message.data, packet.payload, sizeof(packet.payload));
  		tolayer5(0, message);
  		seq_expect_recv_A++;
  		/* Debug output */
  		if (DEBUG)
  			print_pkt("Accpeted at A", packet);
      last_accepted_packet_A = packet;
      /* Send ACK to B side */
      struct pkt ackpkt;
      ackpkt.isACK = 1;
      ackpkt.acknum = packet.seqnum;
      ackpkt.checksum = 0;
      ackpkt.checksum = compute_check_sum(ackpkt);
      last_sent_from_A = ackpkt;
      tolayer3(0, ackpkt);
    } else if (packet.seqnum != seq_expect_recv_A) {
      printf(YEL);
      printf("Received unexpected seqnum.\n");
      printf("Previous ACK probably didn't arrive.\n");
      printf("Resent ACK to A.\n");
      printf(RESET);
      struct pkt ackpkt;
      ackpkt.isACK = 1;
      ackpkt.acknum = last_accepted_packet_A.seqnum;
      ackpkt.checksum = 0;
      ackpkt.checksum = compute_check_sum(ackpkt);
      last_sent_from_A = ackpkt;
      tolayer3(0, ackpkt);
    } else {
      exit(1);
    }
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
  printf(YEL);
  printf("Go back to %d\n", sender_buffer_A[base_A % 50].seqnum);

  for (int i = base_A; i < (base_A + window_A); i++) {
    printf(YEL);
    printf("Retransmitted packet seqnum %d\n", sender_buffer_A[i % 50].seqnum);
    tolayer3(0, sender_buffer_A[i % 50]);
  }

  printf(RESET);

  if (ret_A == 0) {
    time_ret_pkt_sentA = time;
    ret_A = 1;
  }
	starttimer(0, TIME_OUT);
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
  seq_expect_send_A = 20;
  seq_expect_recv_A = 10;
	is_waiting_A = 0;
  //time_ret_pkt_sentA = 0;
  ret_A = 0;
  total_received_ACKs = 0;
  base_A = 0;
  next_open_A = 0;
  window_A = 0;
  buffer_A = 0;
}


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(packet)
struct pkt packet;
{
    if (DEBUG)
		print_pkt("Received at B", packet);

    int ans_checksum = packet.checksum;
    packet.checksum = 0;
    if (compute_check_sum(packet) != ans_checksum) {
          printf(RED);
          print_pkt("Checksum error at B", packet);
          printf(RESET);
          struct pkt nakpkt;
          nakpkt.acknum = -1;
          nakpkt.isACK = 1;
          nakpkt.checksum = 0;
          nakpkt.checksum = compute_check_sum(nakpkt);
          printf(YEL);
          printf("Sent NAK from B\n");
          printf(RESET);
          //last_sent_from_B = nakpkt;
          tolayer3(1, nakpkt);
      return;
    }
    packet.checksum = ans_checksum;

    if(packet.isACK == 1) {

          if (packet.acknum >= sender_buffer_B[base_B % 50].seqnum) {	/* ACK */
            stoptimer(1);
            if (ret_B == 1) {
              printf(GRN);
              printf("B just received ACK from A for a packet previously retransmitted at time %f\n", time_ret_pkt_sentB);
              printf(RESET);
              ret_B = 0;
            }
            printf(GRN);
            printf("Base B seqnum is %d\n", sender_buffer_B[base_B % 50].seqnum);
            for (int i = sender_buffer_B[base_B % 50].seqnum; i <= packet.acknum; i++) {
              total_received_ACKs++;
              base_B = (base_B + 1) % 50;
              buffer_B--;
              window_B--;
              printf("Total successful ACKs: %d\n", total_received_ACKs);
            }
            if(window_B > 0) {
              starttimer(1, TIME_OUT);
            }
            printf(RESET);
            is_waiting_B = 0;
        } else if(packet.acknum > 0 && packet.acknum < sender_buffer_B[base_B % 50].seqnum) {
          printf(YEL);
          printf("Received ACK %d when base B seqnum is %d. Ignore\n", packet.acknum, sender_buffer_B[base_B % 50].seqnum);
          printf(RESET);
        } else if (packet.acknum == -1) {		/* NAK */

          printf(YEL);
          printf("Received NAK\n");
          if (window_B > 0) {
            stoptimer(1);
            printf("Go back to %d\n", sender_buffer_B[base_B % 50].seqnum);
            for (int i = base_B; i < (base_B + window_B); i++) {
              printf(YEL);
              printf("Retransmitted packet seqnum %d\n", sender_buffer_B[i % 50].seqnum);
              tolayer3(1, sender_buffer_B[i % 50]);
            }
            starttimer(1, TIME_OUT);
          } else {
            printf("Empty window. Resending last sent ACK with acknum %d\n", last_sent_from_B.acknum);
            tolayer3(1, last_sent_from_B);
          }
          printf(RESET);

        }
    } else if (packet.seqnum == seq_expect_recv_B) {
  		/* Pass data to layer5 */
  		struct msg message;
  		memcpy(message.data, packet.payload, sizeof(packet.payload));
  		tolayer5(1, message);
  		seq_expect_recv_B++;
  		/* Debug output */
  		if (DEBUG)
  			print_pkt("Accpeted at B", packet);
      last_accepted_packet_B = packet;
      /* Send ACK to A side */
      struct pkt ackpkt;
      ackpkt.isACK = 1;
      ackpkt.acknum = packet.seqnum;
      ackpkt.checksum = 0;
      ackpkt.checksum = compute_check_sum(ackpkt);
      last_sent_from_B = ackpkt;
      tolayer3(1, ackpkt);
    } else if (packet.seqnum != seq_expect_recv_B) {
      printf(YEL);
      printf("Received unexpected seqnum.\n");
      printf("Previous ACk probably didn't arrive.\n");
      printf("Resent ACK to A.\n");
      printf(RESET);
      struct pkt ackpkt;
      ackpkt.isACK = 1;
      ackpkt.acknum = last_accepted_packet_B.seqnum;
      ackpkt.checksum = 0;
      ackpkt.checksum = compute_check_sum(ackpkt);
      last_sent_from_B = ackpkt;
      tolayer3(1, ackpkt);
    } else {
      exit(1);
    }
}

/* called when B's timer goes off */
void B_timerinterrupt()
{
  printf(YEL);
  printf("Go back to %d\n", sender_buffer_B[base_B % 50].seqnum);

  for (int i = base_B; i < (base_B + window_B); i++) {
    printf(YEL);
    printf("Retransmitted packet seqnum %d\n", sender_buffer_B[i % 50].seqnum);
    tolayer3(1, sender_buffer_B[i % 50]);
  }

  printf(RESET);
  if(ret_B == 0) {
    time_ret_pkt_sentB = time;
    ret_B = 1;
  }
  starttimer(1, TIME_OUT);
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
  seq_expect_send_B = 10;
  seq_expect_recv_B = 20;
	is_waiting_B = 0;
  //time_ret_pkt_sentA = 0;
  ret_B = 0;
  base_B = 0;
  next_open_B = 0;
  window_B = 0;
  buffer_B = 0;
}
/*****************************************************************
***************** NETWORK EMULATION CODE STARTS BELOW ***********
The code below emulates the layer 3 and below network environment:
  - emulates the tranmission and delivery (possibly with bit-level corruption
    and packet loss) of packets across the layer 3/4 interface
  - handles the starting/stopping of a timer, and generates timer
    interrupts (resulting in calling students timer handler).
  - generates message to be sent (passed from later 5 to 4)

THERE IS NOT REASON THAT ANY STUDENT SHOULD HAVE TO READ OR UNDERSTAND
THE CODE BELOW.  YOU SHOULD NOT TOUCH, OR REFERENCE (in your code) ANY
OF THE DATA STRUCTURES BELOW.  If you're interested in how I designed
the emulator, you're welcome to look at the code - but again, you should have
to, and you definitely should not have to modify
******************************************************************/

int main()
{
   struct event *eventptr;
   struct msg  msg2give;
   struct pkt  pkt2give;

   int i,j;
   char c;

   init();
   A_init();
   B_init();

   while (1) {
        eventptr = evlist;            /* get next event to simulate */
        if (eventptr==NULL)
           goto terminate;
        evlist = evlist->next;        /* remove this event from event list */
        if (evlist!=NULL)
           evlist->prev=NULL;
        if (TRACE>=2) {
           printf("\nEVENT time: %f,",eventptr->evtime);
           printf("  type: %d",eventptr->evtype);
           if (eventptr->evtype==0)
	       printf(", timerinterrupt  ");
             else if (eventptr->evtype==1)
               printf(", fromlayer5 ");
             else
	     printf(", fromlayer3 ");
           printf(" entity: %d\n",eventptr->eventity);
           }
        time = eventptr->evtime;        /* update time to next event time */
        if (nsim==nsimmax)
	  break;                        /* all done with simulation */
        if (eventptr->evtype == FROM_LAYER5 ) {
            generate_next_arrival();   /* set up future arrival */
            /* fill in msg to give with string of same letter */
            j = nsim % 26;
            for (i=0; i<20; i++)
               msg2give.data[i] = 97 + j;
            if (TRACE>2) {
               printf("          MAINLOOP: data given to student: ");
                 for (i=0; i<20; i++)
                  printf("%c", msg2give.data[i]);
               printf("\n");
	     }
            nsim++;
            if (eventptr->eventity == A)
               A_output(msg2give);
             else
               B_output(msg2give);
            }
          else if (eventptr->evtype ==  FROM_LAYER3) {
            pkt2give.seqnum = eventptr->pktptr->seqnum;
            pkt2give.acknum = eventptr->pktptr->acknum;
            pkt2give.isACK = eventptr->pktptr->isACK;
            pkt2give.checksum = eventptr->pktptr->checksum;
            for (i=0; i<20; i++)
                pkt2give.payload[i] = eventptr->pktptr->payload[i];
	    if (eventptr->eventity ==A)      /* deliver packet by calling */
   	       A_input(pkt2give);            /* appropriate entity */
            else
   	       B_input(pkt2give);
	    free(eventptr->pktptr);          /* free the memory for packet */
            }
          else if (eventptr->evtype ==  TIMER_INTERRUPT) {
            if (eventptr->eventity == A)
	       A_timerinterrupt();
             else
	       B_timerinterrupt();
             }
          else  {
	     printf("INTERNAL PANIC: unknown event type \n");
             }
        free(eventptr);
        }

terminate:
   printf(" Simulator terminated at time %f\n after sending %d msgs from layer5\n",time,nsim);
   return 0;
}



void init()                         /* initialize the simulator */
{
  int i;
  float sum, avg;
  float jimsrand();


   printf("-----  Network Simulator Version 1.1 -------- \n\n");
   printf("Enter the number of messages to simulate: ");
   scanf("%d",&nsimmax);
   printf("Enter  packet loss probability [enter 0.0 for no loss]:");
   scanf("%f",&lossprob);
   printf("Enter packet corruption probability [0.0 for no corruption]:");
   scanf("%f",&corruptprob);
   printf("Enter average time between messages from sender's layer5 [ > 0.0]:");
   scanf("%f",&lambda);
   printf("Enter TRACE:");
   scanf("%d",&TRACE);

   srand(9999);              /* init random number generator */
   sum = 0.0;                /* test random number generator for students */
   for (i=0; i<1000; i++)
      sum=sum+jimsrand();    /* jimsrand() should be uniform in [0,1] */
   avg = sum/1000.0;
   if (avg < 0.25 || avg > 0.75) {
    printf("It is likely that random number generation on your machine\n" );
    printf("is different from what this emulator expects.  Please take\n");
    printf("a look at the routine jimsrand() in the emulator code. Sorry. \n");
    exit(1);
    }

   ntolayer3 = 0;
   nlost = 0;
   ncorrupt = 0;

   time=0.0;                    /* initialize time to 0.0 */
   generate_next_arrival();     /* initialize event list */
}

/****************************************************************************/
/* jimsrand(): return a float in range [0,1].  The routine below is used to */
/* isolate all random number generation in one location.  We assume that the*/
/* system-supplied rand() function return an int in therange [0,mmm]        */
/****************************************************************************/
float jimsrand()
{
  double mmm = RAND_MAX;   /* largest int  - MACHINE DEPENDENT!!!!!!!!   */
  float x;                   /* individual students may need to change mmm */
  x = rand()/mmm;            /* x should be uniform in [0,1] */
  return(x);
}

/********************* EVENT HANDLINE ROUTINES *******/
/*  The next set of routines handle the event list   */
/*****************************************************/

void generate_next_arrival()
{
   double x;
   struct event *evptr;
   float ttime;
   int tempint;

   if (TRACE>2)
       printf("          GENERATE NEXT ARRIVAL: creating new arrival\n");

   x = lambda*jimsrand()*2;  /* x is uniform on [0,2*lambda] */
                             /* having mean of lambda        */
   evptr = (struct event *)malloc(sizeof(struct event));
   evptr->evtime =  time + x;
   evptr->evtype =  FROM_LAYER5;
   if (BIDIRECTIONAL && (jimsrand()>0.5) )
      evptr->eventity = B;
    else
      evptr->eventity = A;
   insertevent(evptr);
}


void insertevent(struct event *p)
{
   struct event *q,*qold;

   if (TRACE>2) {
      printf("            INSERTEVENT: time is %lf\n",time);
      printf("            INSERTEVENT: future time will be %lf\n",p->evtime);
      }
   q = evlist;     /* q points to header of list in which p struct inserted */
   if (q==NULL) {   /* list is empty */
        evlist=p;
        p->next=NULL;
        p->prev=NULL;
        }
     else {
        for (qold = q; q !=NULL && p->evtime > q->evtime; q=q->next)
              qold=q;
        if (q==NULL) {   /* end of list */
             qold->next = p;
             p->prev = qold;
             p->next = NULL;
             }
           else if (q==evlist) { /* front of list */
             p->next=evlist;
             p->prev=NULL;
             p->next->prev=p;
             evlist = p;
             }
           else {     /* middle of list */
             p->next=q;
             p->prev=q->prev;
             q->prev->next=p;
             q->prev=p;
             }
         }
}

void printevlist()
{
  struct event *q;
  int i;
  printf("--------------\nEvent List Follows:\n");
  for(q = evlist; q!=NULL; q=q->next) {
    printf("Event time: %f, type: %d entity: %d\n",q->evtime,q->evtype,q->eventity);
    }
  printf("--------------\n");
}



/********************** Student-callable ROUTINES ***********************/

/* called by students routine to cancel a previously-started timer */
void stoptimer(int AorB)
{
 struct event *q,*qold;

 if (TRACE>2)
    printf("          STOP TIMER: stopping timer at %f\n",time);
/* for (q=evlist; q!=NULL && q->next!=NULL; q = q->next)  */
 for (q=evlist; q!=NULL ; q = q->next)
    if ( (q->evtype==TIMER_INTERRUPT  && q->eventity==AorB) ) {
       /* remove this event */
       if (q->next==NULL && q->prev==NULL)
             evlist=NULL;         /* remove first and only event on list */
          else if (q->next==NULL) /* end of list - there is one in front */
             q->prev->next = NULL;
          else if (q==evlist) { /* front of list - there must be event after */
             q->next->prev=NULL;
             evlist = q->next;
             }
           else {     /* middle of list */
             q->next->prev = q->prev;
             q->prev->next =  q->next;
             }
       free(q);
       return;
     }
  printf("Warning: unable to cancel your timer. It wasn't running.\n");
  exit(1);
}


void starttimer(int AorB, float increment)
{

 struct event *q;
 struct event *evptr;

 if (TRACE>2)
    printf("          START TIMER: starting timer at %f\n",time);
 /* be nice: check to see if timer is already started, if so, then  warn */
/* for (q=evlist; q!=NULL && q->next!=NULL; q = q->next)  */
   for (q=evlist; q!=NULL ; q = q->next)
    if ( (q->evtype==TIMER_INTERRUPT  && q->eventity==AorB) ) {
      printf("Warning: attempt to start a timer that is already started\n");
      exit(1);
      return;
      }

/* create future event for when timer goes off */
   evptr = (struct event *)malloc(sizeof(struct event));
   evptr->evtime =  time + increment;
   evptr->evtype =  TIMER_INTERRUPT;
   evptr->eventity = AorB;
   insertevent(evptr);
}


/************************** TOLAYER3 ***************/
void tolayer3(int AorB, struct pkt packet)
{
 struct pkt *mypktptr;
 struct event *evptr,*q;
 float lastime, x;
 int i;


 ntolayer3++;

 /* simulate losses: */
 if (jimsrand() < lossprob)  {
      nlost++;
      if (TRACE>0) {
        printf(RED);
        printf("          TOLAYER3: packet being lost\n");
        printf(RESET);
      }

      return;
    }

/* make a copy of the packet student just gave me since he/she may decide */
/* to do something with the packet after we return back to him/her */
 mypktptr = (struct pkt *)malloc(sizeof(struct pkt));
 mypktptr->seqnum = packet.seqnum;
 mypktptr->acknum = packet.acknum;
 mypktptr->checksum = packet.checksum;
 mypktptr->isACK = packet.isACK;

 for (i=0; i<20; i++)
    mypktptr->payload[i] = packet.payload[i];
 if (TRACE>2)  {
   printf("          TOLAYER3: seq: %d, ack %d, check: %d ", mypktptr->seqnum,
	  mypktptr->acknum,  mypktptr->checksum);
    for (i=0; i<20; i++)
        printf("%c",mypktptr->payload[i]);
    printf("\n");
   }

/* create future event for arrival of packet at the other side */
  evptr = (struct event *)malloc(sizeof(struct event));
  evptr->evtype =  FROM_LAYER3;   /* packet will pop out from layer3 */
  evptr->eventity = (AorB+1) % 2; /* event occurs at other entity */
  evptr->pktptr = mypktptr;       /* save ptr to my copy of packet */
/* finally, compute the arrival time of packet at the other end.
   medium can not reorder, so make sure packet arrives between 1 and 10
   time units after the latest arrival time of packets
   currently in the medium on their way to the destination */
 lastime = time;
/* for (q=evlist; q!=NULL && q->next!=NULL; q = q->next) */
 for (q=evlist; q!=NULL ; q = q->next)
    if ( (q->evtype==FROM_LAYER3  && q->eventity==evptr->eventity) )
      lastime = q->evtime;
 evptr->evtime =  lastime + 1 + 9*jimsrand();



 /* simulate corruption: */
 if (jimsrand() < corruptprob)  {
    ncorrupt++;
    if ( (x = jimsrand()) < .75)
       mypktptr->payload[0]='Z';   /* corrupt payload */
      else if (x < .875)
       mypktptr->seqnum = 999999;
      else if (x < .925)
          mypktptr->isACK = 999999;
      else
       mypktptr->acknum = 999999;
    if (TRACE>0){
        printf(RED);
        printf("          TOLAYER3: packet being corrupted\n");
        printf(RESET);
    }

    }

  if (TRACE>2)
     printf("          TOLAYER3: scheduling arrival on other side\n");
  insertevent(evptr);
}

void tolayer5(int AorB, struct msg datasent)
{
  int i;
  if (TRACE>2) {
     printf("          TOLAYER5: data received: ");
     for (i=0; i<20; i++)
        printf("%c",datasent.data[i]);
     printf("\n");
   }

}
