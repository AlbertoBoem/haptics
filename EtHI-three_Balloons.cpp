/*****************************************************************************************************

Control program for an Encoutred-type Haptic Interface
******************************************************************************************************/

#include<windows.h>
#include<stdio.h>
#include<stdlib.h>
#include<conio.h>
#include<string.h>
#include<math.h>
#include<time.h>
#include"SC02.h"
#include"CM001.h"
#include"Measurement.h"
#include<fstream>
#include<iostream>
//#include

// ----------------------------------- OSC ------------------------------------------------
#include <cstring>
#include <thread>
#include <mutex>


#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

// ----osc including the path for the files ---------------------
#ifdef _DEBUG
#pragma comment (lib, "oscpack_d.lib")
#else
#pragma comment (lib, "oscpack.lib")
#endif

#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "winmm.lib")

//--------------------------- IP - PORT - BUFFER ---------------------------------------

#define ADDRESS "192.168.10.2"//"192.168.10.2" //localhost                    <--------------------------------------------
#define SEND_PORT 7000 //7000 //12345                                         <--------------------------------------------
#define RECV_PORT 7000 //7000 //12345 // same port for send and receive       <--------------------------------------------
#define OUTPUT_BUFFER_SIZE 1024  //                                           <--------------------------------------------

//---------------------------------- OSC -----------------------------------------------------

#pragma comment (lib,"winmm.lib")

COORD Pos;
HANDLE hOut; 

CM_PARAM Param ={
	//   dt[s], fCtrlTimeout fComTimeout
	0.001f, 1.00f, 1.000f,
};

//------------------------------- OSC -----------------------------
// Struct Pointer

UdpListeningReceiveSocket *s1;
UdpTransmitSocket *socketTransmit;


//-----------------------------------------------------------------

const int CM_NUM=6;
SC02 ScModule;
CM001 Module[CM_NUM];
const int limit_max=10000;
const int limit_min=0;

//���W���[���P
int Val_v1 = 0, Val_r1 = 0, Val_p1 = 0, motor11 = 0, motor12 = 0;
int Val_v1_m = 0, Val_v1_pre = 0, volume1 = 0, vc1 = 0, vc1f = 0; //operate volume
int Val_p1_m = 0, Val_p1_pre = 0, posi1 = 0, pc1 = 0, pc1f = 0;   //operate position
double atm1 = 0, atm1_v = 0, atm1_pa = 0, r1_k = 0.4;             //operate rigidity
int p1 = 0, point1 = 0, key1 = 0, dig1 = 0;
int enc11, enc12, sen1, pre_sen1;
int enc12_p, enc12_d, enc12_i, m_enc12 = 0, pre_enc12 = 0;        //rotate axial direction
double kp12 = 9.0, kd12 = 20.0, ki12 = 0.0001;
int enc11_p, enc11_d, enc11_i, m_enc11 = 0, pre_enc11 = 0;        //bend tube
double kp11_f = 0.3, kd11_f = 0.15, ki11_f = 0.00005;
double kp11_b = 0.15, kd11_b = 0.3, ki11_b = 0.00005;

//���W���[���Q
int Val_v2 = 0, Val_r2 = 0, Val_p2 = 0, motor21 = 0, motor22 = 0;
int Val_v2_m = 0, Val_v2_pre = 0, volume2 = 0, vc2 = 0, vc2f = 0; //operate volume
int Val_p2_m = 0, Val_p2_pre = 0, posi2 = 0, pc2 = 0, pc2f = 0;   //operate position
double atm2 = 0, atm2_v = 0, atm2_pa = 0, r2_k = 0.4;             //operate rigidity
int p2 = 0, point2 = 0, key2 = 0, dig2 = 0;
int enc21, enc22, sen2, pre_sen2;
int enc22_p, enc22_d, enc22_i, m_enc22 = 0, pre_enc22 = 0;        //rotate axial direction
double kp22 = 9.0, kd22 = 20.0, ki22 = 0.0001;
int enc21_p, enc21_d, enc21_i, m_enc21 = 0, pre_enc21 = 0;        //bend tube
double kp21_f = 0.3, kd21_f = 0.15, ki21_f = 0.00005;
double kp21_b = 0.15, kd21_b = 0.3, ki21_b = 0.00005;

//���W���[���R
int Val_v3 = 0, Val_r3 = 0, Val_p3 = 0, motor31 = 0, motor32 = 0;
int Val_v3_m = 0, Val_v3_pre = 0, volume3 = 0, vc3 = 0, vc3f = 0; //operate volume
int Val_p3_m = 0, Val_p3_pre = 0, posi3 = 0, pc3 = 0, pc3f = 0;   //operate position
double atm3 = 0, atm3_v = 0, atm3_pa = 0, r3_k = 0.4;             //operate rigidity
int p3 = 0, point3 = 0, key3 = 0, dig3 = 0;
int enc32_p, enc32_d, enc32_i, m_enc32 = 0, pre_enc32 = 0;        //rotate axial direction
double kp32 = 9.0, kd32 = 20.0, ki32 = 0.0001;
int enc31, enc32, sen3, pre_sen3;
int enc31_p, enc31_d, enc31_i, m_enc31 = 0, pre_enc31 = 0;        //bend tube
double kp31_f = 0.3, kd31_f = 0.15, ki31_f = 0.00005;
double kp31_b = 0.15, kd31_b = 0.3, ki31_b = 0.00005;

//�e�X�g�p
int p_demo = 0, p_demo_frag = 0;
int r_demo = 0, r_demo_frag = 0;
int h_demo = 0, h_demo_frag = 0;
int v_demo = 0, v_demo_frag = 0;
int k_demo = 0, k_demo_frag = 0;

bool checkIt = 1;

//for reduce rigidity ---> Rigidity_ctr123
int checkDown1;
int checkDown2;
int checkDown3;



//--------------------------------------------- OSC ----------------------------------------

// ---> DEFINE (1) defining the type of data to decode when receiving
typedef struct MyDATA
{
    // ---- TEST-------
	//float a;
	//float b;
	// ----------------

	//Send
	//int pSens1, pSens2, pSens3; //pressure sensor

	//--> Receive
	float rCtrl1; // , rCtrl2, rCtrl3; //rigidity control
	float rCtrl2;
	float rCtrl3;

	float vCtrl1;
	float vCtrl2;
	float vCtrl3;

	float pCtrl1;
	float pCtrl2;
	float pCtrl3;

	float rotCtrl1;
	float rotCtrl2;
	float rotCtrl3;

	float bCtrl1;
	float bCtrl2;
	float bCtrl3;

	//Check conditions
	float vol123;
	float vertical123;
	float rigi123;

	//Test Condition Reduce Rigidity
	float down1;
	float down2;
	float down3;

	//Global Condition
	float globalCheck;

};

//-------------------------------------------------- Receiving ------------------------------------------

class ExamplePacketListener : public osc::OscPacketListener {

protected:
	std::mutex sync;
	bool updated = false;
	//char recv_data;
	MyDATA recv_data;
	//float recv_data;
	virtual void ProcessMessage(const osc::ReceivedMessage& m,
		const IpEndpointName& remoteEndpoint)
	{
		(void)remoteEndpoint; // suppress unused parameter warning

		try {

			//-------------------------------------------- DEFINE (3) ----------------------------------

			/*
			if (std::strcmp(m.AddressPattern(), "/test") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.a = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}                                         // AsFloatUnchecked , AsInt32Unchecked ....
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}*/
			//add other messages to decode
			/*
			else if (std::strcmp(m.AddressPattern(), "/test") == 0) {
			}*/

			//----------------------- TEST -----------------
			/*
			else if(std::strcmp(m.AddressPattern(), "/test1") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.b = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}*/


			//------------------------------------------RECEIVE---------------------------------------

			// ---> RIGIDITY
			if (std::strcmp(m.AddressPattern(), "/rigidity1") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.rCtrl1 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/rigidity2") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.rCtrl2 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/rigidity3") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.rCtrl3 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			// ---> VOLUME
			else if (std::strcmp(m.AddressPattern(), "/volumec1") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.vCtrl1 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/volumec2") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.vCtrl2 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/volumec3") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.vCtrl3 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			// ---> VERTICAL POSITION
			else if (std::strcmp(m.AddressPattern(), "/posc1") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.pCtrl1 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/posc2") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.pCtrl2 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/posc3") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.pCtrl3 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

            // ---> ROTATION
			else if (std::strcmp(m.AddressPattern(), "/rotc1") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.rotCtrl1 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/rotc2") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.rotCtrl2 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/rotc3") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.rotCtrl3 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			// ---> BEND
			else if (std::strcmp(m.AddressPattern(), "/bendc1") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.bCtrl1 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/bendc2") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.bCtrl2 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			else if (std::strcmp(m.AddressPattern(), "/bendc3") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.bCtrl3 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			// ---> CONDITIONS TO CHECK

			// Rigidity 1 2 3
			else if (std::strcmp(m.AddressPattern(), "/key") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.rigi123 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			// Volume 1 2 3
			else if (std::strcmp(m.AddressPattern(), "/vcf") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.vol123 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			//Vertical Pos 1 2 3
			else if (std::strcmp(m.AddressPattern(), "/global") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.globalCheck = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			//Reduce Rigidity Check 1
			else if (std::strcmp(m.AddressPattern(), "/d1") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.down1 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			//Reduce Rigidity Check 2
			else if (std::strcmp(m.AddressPattern(), "/d2") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.down2 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}

			//Reduce Rigidity Check 3
 			else if (std::strcmp(m.AddressPattern(), "/d3") == 0) {

				std::unique_lock<std::mutex> lock(sync);
				osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
				//std::cout << arg->TypeTag() << std::endl;
				if (arg != m.ArgumentsEnd()){
					recv_data.down3 = arg->AsFloatUnchecked(); //member function AsType() bool,char,float,int....select the type
				}
				arg++;

				//(arg++)->AsFloat(); //next message
				updated = true;
				//std::cout << "data=" << recv_data << "        \n";
			}


		}
		catch (osc::Exception& e) { //it was commented out...
			std::cout << "error while parsing message: "
				<< m.AddressPattern() << ": " << e.what() << "\n";
		}
	}


// ---> DEFINE (2) --- The type of data that we will receive

public:
	void getData(MyDATA &d) { //char
		std::unique_lock<std::mutex> lock(sync);
		updated = false;

		//Test
		d.rCtrl1 = recv_data.rCtrl1;
		d.rCtrl2 = recv_data.rCtrl2;
		d.rCtrl3 = recv_data.rCtrl3;

		d.vCtrl1 = recv_data.vCtrl1;
		d.vCtrl2 = recv_data.vCtrl2;
		d.vCtrl3 = recv_data.vCtrl3;

		d.pCtrl1 = recv_data.pCtrl1;
		d.pCtrl2 = recv_data.pCtrl2;
		d.pCtrl3 = recv_data.pCtrl3;

		d.rotCtrl1 = recv_data.rotCtrl1;
		d.rotCtrl2 = recv_data.rotCtrl2;
		d.rotCtrl3 = recv_data.rotCtrl3;

		d.bCtrl1 = recv_data.bCtrl1;
		d.bCtrl2 = recv_data.bCtrl2;
		d.bCtrl3 = recv_data.bCtrl3;

		//Conditions
		d.rigi123 = recv_data.rigi123; //rigidity
		d.vol123 = recv_data.vol123; //volume
		//d.vertical123 = recv_data.vertical123; //vertical movement

		//Reduce Rigidity check
		d.down1 = recv_data.down1;
		d.down2 = recv_data.down2;
		d.down3 = recv_data.down3;

		//Global check
		d.globalCheck = recv_data.globalCheck;


		//d.b = recv_data.b;

		//Receive
		//d.rCtrl1; recv_data.rCtrl1; //rigidity control
		//d.rCtrl2; recv_data.rCtrl2; //rigidity control
		//d.rCtrl3; recv_data.rCtrl3; //rigidity control

		return;
	}
	bool isUpdated() {
		std::unique_lock<std::mutex> lock(sync);
		return updated;
	}
};

void recv_thread(UdpListeningReceiveSocket *s) {
	s->RunUntilSigInt();
	std::cout << "finish" << std::endl;
}


//-------------------------------------------------------------------------------------------


BOOL TestSC( void )
{

	//	if( !ScModule.SetID( 1 ) )	return( FALSE );
	if( !ScModule.GetID() ) return( FALSE );
	printf("ID = %d\r\n", ScModule.bID );
	/*
	if( !ScModule.GetVersion() ) return( FALSE );
	//	printf("%x\r\n", ScModule.bVersion );
	//	printf("Status %d \r\n", ScModule.bStatus );
	/**/
	/*
	printf("Hit any key\r\n");
	getch();
	if( !ScModule.Blink(3) ) return( FALSE );

	printf("Status %d \r\n", ScModule.bStatus );
	/**/
	return( TRUE );
}

BOOL TestCM( void )
{
	Module[0].GetVersion();
	printf( "CM001 Ver.%d.%d\r\n", 0x0F & Module[0].bVersion >>4, 0x0F & Module[0].bVersion );

	if( !Module[0].Ping() ) return( FALSE );
	printf("Ping OK\r\n", ScModule.bStatus, Module[0].bStatus );

	_getch();
	Module[0].Blink( 2 );
	printf("status SC02 %d CM001 %d\r\n", ScModule.bStatus, Module[0].bStatus );

	return( TRUE );
}

void SetData( CM001 *pModule, int iVal, int jVal, int kVal )
{

	// �|�[�gA��PIO�̏ꍇ�̃f�[�^�ݒ�
	if( pModule->bBoardID_PA == CM001_PIO_BOARD ){
		// �o�C�g�P�ʂŐݒ肷���ꍇ�iPIOA��8bit�܂ŗL���j
		//pModule->PioBoardA.Word.wData = ~pModule->PioBoardA.Word.wData;
		pModule->PioBoardA.Word.wDir = 0xFFFF;
		// �r�b�g�P�ʂŐݒ肷���ꍇ
		pModule->PioBoardA.Bit.P1 = ~Module[0].PioBoardA.Bit.P1;
		pModule->PioBoardA.Bit.DIR1 = 0;
	}

	// �|�[�gB�̃f�[�^�ݒ�

	switch(  pModule->bBoardID_PB ){
	case CM001_PWM_BOARD:
//#ifdef STM
		pModule->stm_ctl.ref0 = iVal;
		pModule->stm_ctl.ref1 = jVal;
		pModule->stm_ctl.ref2 = kVal;
		//	printf("STM %d ", pModule->stm_ctl.ref0);
//#else
		pModule->PwmBoard.sPwm1 = iVal;
		pModule->PwmBoard.sPwm2 = jVal;
		//printf("sPwm %d %d ", Module[0].PwmBoard.sPwm1, Module[0].PwmBoard.sPwm2 );
//#endif
		break;
	case CM001_DA_BOARD:
		pModule->DaBoard.sDA = iVal;
		printf("da %d ", pModule->DaBoard.sDA );
		break;
	case CM001_PIO_BOARD:
		// �o�C�g�P�ʂŐݒ肷���ꍇ�iPIOB��10bit�܂ŗL���j
		//Module[0].PioBoardB.Word.wData = ~Module[0].PioBoardB.Word.wData;
		pModule->PioBoardB.Word.wDir = 0xFFFF;
		// �r�b�g�P�ʂŐݒ肷���ꍇ
		pModule->PioBoardB.Bit.P9 = ~pModule->PioBoardB.Bit.P9;
		pModule->PioBoardB.Bit.DIR9 = 0;
		break;
	}

}

void IndecateData( CM001 *pModule )
{
	switch( pModule->bBoardID_PA){
	case CM001_ENC_BOARD:
//#ifdef STM
		//printf("stm %d ad %d ", pModule->stm_state.lPos, pModule->EncBoard.wSensor[0]);
//#else
		//printf("enc %d ad %d ", pModule->EncBoard.lEncoder, pModule->EncBoard.wSensor[0] );
//#endif
		break;
	case CM001_SENSOR_BOARD:
		/*printf("\r\nsens %d  %d  %d  %d  %d  %d  %d  %d",
			pModule->SensorBoard.wSensor[0], pModule->SensorBoard.wSensor[1],
			pModule->SensorBoard.wSensor[2], pModule->SensorBoard.wSensor[3],
			pModule->SensorBoard.wSensor[4], pModule->SensorBoard.wSensor[5],
			pModule->SensorBoard.wSensor[6], pModule->SensorBoard.wSensor[7]
		);*/
		break;
	case CM001_PIO_BOARD:
		printf("pioA %04x ", pModule->PioBoardA.Word.wData );
		break;
	}
	if( pModule->bBoardID_PB == CM001_PIO_BOARD ){
		printf("pioB %04x ", pModule->PioBoardB.Word.wData );
	}
	printf("\r\n");

}

void Balloon_bend_ctr1(void){//Unit 1
	 enc11 = -((int)((Module[0].EncBoard.lEncoder)/10));
	 //difference between present sensor value and value of target
	 enc11_p = m_enc11 - enc11;
	 //differential term
	 enc11_d = enc11 - pre_enc11;
	 //integral term
	 enc11_i += m_enc11 - enc11;
	 //output to DC motor for bending tube[pwm]
	 if (m_enc11 - enc11 >= 5){
		 motor11 = (-1)*((kp11_f * enc11_p) - (kd11_f * enc11_d) + (ki11_f * enc11_i));
	}
	 else if (m_enc11 - enc11 < 5){
		 motor11 = (-1)*((kp11_b * enc11_p) - (kd11_b * enc11_d) + (ki11_b * enc11_i));
	 }
	 pre_enc11 = enc11;
}

void Balloon_bend_ctr2(void){//Unit 2
	enc21 = -((int)(Module[2].EncBoard.lEncoder) / 10);
	//difference between present sensor value and value of target
	enc21_p = m_enc21 - enc21;
	//differential term
	enc21_d = enc21 - pre_enc21;
	//integral term
	enc21_i += m_enc21 - enc21;
	//output to DC motor for bending tube[pwm]
	if (m_enc21 - enc21 >= 5){
		motor21 = (-1)*((kp21_f * enc21_p) - (kd21_f * enc21_d) + (ki21_f * enc21_i));
	}
	else if (m_enc21 - enc21 < 5){
		motor21 = (-1)*((kp21_b * enc21_p) - (kd21_b * enc21_d) + (ki21_b * enc21_i));
	}
	pre_enc21 = enc21;
}

void Balloon_bend_ctr3(void){//Unit 3
	enc31 = -((int)(Module[4].EncBoard.lEncoder) / 10);
	//difference between present sensor value and value of target
	enc31_p = m_enc31 - enc31;
	//differential term
	enc31_d = enc31 - pre_enc31;
	//integral term
	enc31_i += m_enc31 - enc31;
	//output to DC motor for bending tube[pwm]
	if (m_enc31 - enc31 >= 5){
		motor31 = (-1)*((kp31_f * enc31_p) - (kd31_f * enc31_d) + (ki31_f * enc31_i));
	}
	else if (m_enc31 - enc31 < 5){
		motor31 = (-1)*((kp31_b * enc31_p) - (kd31_b * enc31_d) + (ki31_b * enc31_i));
	}
	pre_enc31 = enc31;
}

void Balloon_rot_ctr1(void){
	enc12 = Module[1].EncBoard.lEncoder;
	//difference between present sensor value and value of target
	enc12_p = m_enc12 - enc12;
	//differential term
	enc12_d = enc12 - pre_enc12;
	//integral term
	enc12_i += m_enc12 - enc12;
	//output to DC motor to rotate tube
	motor12 = (-1)*((kp12 * enc12_p) - (kd12 * enc12_d) + (ki12 * enc12_i));

	pre_enc12 = enc12;
}

void Balloon_rot_ctr2(void){
	enc22 = Module[3].EncBoard.lEncoder;
	//difference between present sensor value and value of target
	enc22_p = m_enc22 - enc22;
	//differential term
	enc22_d = enc22 - pre_enc22;
	//integral term
	enc22_i += m_enc22 - enc22;
	//output to DC motor to rotate tube
	motor22 = (-1)*((kp22 * enc22_p) - (kd22 * enc22_d) + (ki22 * enc22_i));

	pre_enc22 = enc22;
}

void Balloon_rot_ctr3(void){
	enc32 = Module[5].EncBoard.lEncoder;
	//difference between present sensor value and value of target
	enc32_p = m_enc32 - enc32;
	//differential term
	enc32_d = enc32 - pre_enc32;
	//integral term
	enc32_i += m_enc32 - enc32;
	//output to DC motor to rotate tube
	motor32 = (-1)*((kp32 * enc32_p) - (kd32 * enc32_d) + (ki32 * enc32_i));

	pre_enc32 = enc32;
}

void Rigidity_ctr1(void){

	//pressure sensor value
	sen1 = Module[3].EncBoard.wSensor[0] - 20;
	//atm1 = sen1 * 0.2 + pre_sen1 * 0.8;//Low-pass filter
	atm1 = sen1;
	pre_sen1 = atm1;
	atm1_v = 0.009*atm1 - 0.899;
	atm1_pa = (atm1_v - 1) * 100 / 4;//internal pressure[Pa]
	//output to liner actuator
	if (key1 == 1){
		//difference between position of target and present position(operate position)
		dig1 = (point1 * 10) - (atm1_pa * 10);

		if (point1>0){    //was if
			if (dig1>2){
				Val_r1 += (dig1 * r1_k);
			}
			else if (dig1<-2){
				Val_r1 -= -(dig1 * r1_k);
			}
		}

	}

}



void Rigidity_ctr2(void){
	//pressure sensor value
	sen2 = Module[4].EncBoard.wSensor[0] - 20;
	atm2 = sen2 * 0.2 + pre_sen2 * 0.8;//Low-pass filter
	pre_sen2 = atm2;
	atm2_v = 0.009*atm2 - 0.899;
	atm2_pa = (atm2_v - 1) * 100 / 4;//internal pressure[Pa]
	//output to liner actuator
	if (key2 == 1){
		//difference between position of target and present position(operate position)
		dig2 = (point2 * 10) - (atm2_pa * 10);

		if (point2>0){ //was if
			if (dig2>2){
				Val_r2 += (dig2 * r2_k);
			}
			else if (dig2<-2){
				Val_r2 -= -(dig2 * r2_k);
			}
		}

	}

}



void Rigidity_ctr3(void){
	//pressure sensor value
	sen3 = Module[5].EncBoard.wSensor[0] - 20;//Low-pass filter
	atm3 = sen3 * 0.2 + pre_sen3 * 0.8;
	pre_sen3 = atm3;
	atm3_v = 0.009*atm3 - 0.899;
	atm3_pa = (atm3_v - 1) * 100 / 4;//internal pressure[Pa]
	//output to liner actuator
	if (key3 == 1){
		//difference between position of target and present position(operate position)
		dig3 = (point3 * 10) - (atm3_pa * 10);
		if (point3>0){
			if (dig3>2){
				Val_r3 += (dig3 * r3_k);
			}
			else if (dig3<-2){
				Val_r3 -= -(dig3 * r3_k);
			}
		}

	}

}



void Volume_ctr1(void){
	//Val_v1_m = (0.6797 * volume1 * volume1) + (88.8609 * volume1) + 253.9235 + 500;//determine target value
	Val_v1_m = (0.6797 * volume1 * volume1) + (88.8609 * volume1) + 253.9235 + 500;//determine target value
	if (vc1<50 && vc1f == 1){
		Val_v1 += (Val_v1_m - Val_v1_pre) / 50;
		vc1++;
		if (vc1 == 50){
			vc1 = 0;
			vc1f = 0;
			Val_v1_pre = Val_v1;
		}
	}
}

void Volume_ctr2(void){
		Val_v2_m = (0.6797 * volume2 * volume2) + (88.8609 * volume2) + 253.9235 + 250;
	if (vc2<50 && vc2f == 1){
		Val_v2 += (Val_v2_m - Val_v2_pre) / 50;
		vc2++;
		if (vc2 == 50){
			vc2 = 0;
			vc2f = 0;
			Val_v2_pre = Val_v2;
		}
	}
}

void Volume_ctr3(void){
	Val_v3_m = (0.6797 * volume3 * volume3) + (88.8609 * volume3) + 253.9235;
	if (vc3<50 && vc3f == 1){
		Val_v3 += (Val_v3_m - Val_v3_pre) / 50;
		vc3++;
		if (vc3 == 50){
			vc3 = 0;
			vc3f = 0;
			Val_v3_pre = Val_v3;
		}
	}
}

void Posi_ctr1(void){
	Val_p1_m = posi1 * 100;
	if (pc1<100 && pc1f == 1){
		Val_p1 += (Val_p1_m - Val_p1_pre) / 100;
		pc1++;
		if (pc1 == 100){
			pc1 = 0;
			//pc1f = 0;
			Val_p1_pre = Val_p1;
		}
	}
}

void Posi_ctr2(void){
	Val_p2_m = posi2 * 100;
	if (pc2<100 && pc2f == 1){
		Val_p2 += (Val_p2_m - Val_p2_pre) / 100;
		pc2++;
		if (pc2 == 100){
			pc2 = 0;
			//pc2f = 0;
			Val_p2_pre = Val_p2;
		}
	}
}

void Posi_ctr3(void){
	Val_p3_m = posi3 * 100;
	if (pc3<100 && pc3f == 1){
		Val_p3 += (Val_p3_m - Val_p3_pre) / 100;
		pc3++;
		if (pc3 == 100){
			pc3 = 0;
			//pc3f = 0;
			Val_p3_pre = Val_p3;
		}
	}
}

void Posi_demo(void){
	if (p_demo_frag == 1){
		if (p_demo < 100){
			m_enc11 = m_enc11 + 40;
			m_enc21 = m_enc21 + 40;
			m_enc31 = m_enc31 + 40;
			p_demo++;
		}
		else if (p_demo>=100 && p_demo < 200){
			m_enc11 = m_enc11 - 40;
			m_enc21 = m_enc21 - 40;
			m_enc31 = m_enc31 - 40;
			p_demo++;
		}
		else if (p_demo == 200){
			p_demo = 0;
			p_demo_frag = 0;
		}

	}
}

void Rot_demo(void){
	if (r_demo_frag == 1){
		if (r_demo < 100){
			m_enc12 = m_enc12 + 1;
			m_enc22 = m_enc22 + 1;
			m_enc32 = m_enc32 + 1;
			r_demo++;
		}
		else if (r_demo >= 100 && r_demo < 300){
			m_enc12 = m_enc12 - 1;
			m_enc22 = m_enc22 - 1;
			m_enc32 = m_enc32 - 1;
			r_demo++;
		}
		else if (r_demo >= 300 && r_demo < 400){
			m_enc12 = m_enc12 + 1;
			m_enc22 = m_enc22 + 1;
			m_enc32 = m_enc32 + 1;
			r_demo++;
		}
		else if (r_demo == 400){
			r_demo = 0;
			r_demo_frag = 0;
		}

	}
}

void High_demo(void){
	if (h_demo_frag == 1){
		if (h_demo < 100){
			posi1 = 30;
			posi2 = 0;
			posi3 = 0;
			h_demo++;
		}
		else if (h_demo >= 100 && h_demo < 200){
			posi1 = 0;
			posi2 = 30;
			posi3 = 0;
			h_demo++;
		}
		else if (h_demo >= 200 && h_demo < 300){
			posi1 = 0;
			posi2 = 0;
			posi3 = 30;
			h_demo++;
		}
		else if (h_demo == 300){
			posi1 = 0;
			posi2 = 0;
			posi3 = 0;
			h_demo = 0;
			//h_demo_frag = 0;
		}

	}

	if (h_demo_frag == 2){
		posi1 = 0;
		posi2 = 0;
		posi3 = 0;
		h_demo = 0;
		h_demo_frag = 0;
	}
}

void Volume_demo(void){
	if (v_demo_frag == 1){
		if (v_demo < 30){
			volume1 = 40;
			volume2 = 40;
			volume3 = 40;
			v_demo++;
		}
		else if (v_demo >= 30 && v_demo < 60){
			//volume2 = 30;
			v_demo++;
		}
		else if (v_demo == 60){
			//volume3 = 30;
			v_demo = 0;
			v_demo_frag = 0;
		}

	}
}

void Rigidity_demo(void){
	if (k_demo_frag == 1){
		if (k_demo < 30){
			point1 = 8;
			point2 = 8;
			point3 = 8;
			k_demo++;
		}
		else if (k_demo >= 30 && k_demo < 60){
			//volume2 = 30;
			k_demo++;
		}
		else if (k_demo == 60){
			//volume3 = 30;
			k_demo = 0;
			k_demo_frag = 0;
		}

	}
}


// ���{�I�ȃf�[�^�̑����M�D���o�̓f�[�^�̓N���XCM001�̃����o�ϐ����g�p
void Exchange(ExamplePacketListener *lis) //pass the global pointer
{

	int iVal=0,iVal2=0,iVal_pre=0,sync=0,async=0,i=0;
	int a = 0;

	//-------- osc test value --------------
	int val1 = 1;
	float val2 = 2.1;
	//--------------------------------

	BOOL loop=TRUE,wait_f=FALSE;
	DWORD start;
	DWORD stop;
	int direction,peek_ctr = 0;
	int tmp_sum=0;
	int table_ctr = 0;
	int re_cal;
	FILE *fp;
	time_t timer;
	struct tm *t_st;
	char filename[128];
	BOOL edge_f = FALSE; //�l���ς����Ă���BUSY�ɂȂ������ǂ���
	MES_StartTimeCount();
	hOut = GetStdHandle(STD_OUTPUT_HANDLE);

	while(loop){

		//point1 += 10;
		//key1 = 1;

		// --------------------------------------- RECEIVE DATA -----------------------------------------------
		//if (lis->isUpdated()) {
			MyDATA dato;
			//char data;
			lis->getData(dato);
			//std::cout << "data=" << dato.a << " " << dato.b << "        \n";
			//std::cout << "data = " << dato.rCtrl1 << " " << dato.rCtrl2 << " " << dato.rCtrl3 << "    \n";
		//}
	   //---------------------------------------------------------------------------------------------------------


	   // --------------------------------   Received Control Parameters   ---------------------------

			// --> Rigidity Ctrl
			int controlRigidity1 = (int)dato.rCtrl1;
			//controlRigidity1 = point1;

			int controlRigidity2 = (int)dato.rCtrl2;
			//controlRigidity2 = point2;

			int controlRigidity3 = (int)dato.rCtrl3;
			//controlRigidity3 = point3;


			// --> Volume Ctrl
			int volumeControl1 = (int)dato.vCtrl1;
			//volumeControl1 = volume1;

			int volumeControl2 = (int)dato.vCtrl2;
			//volumeControl2 = volume2;

			int volumeControl3 = (int)dato.vCtrl3;
			//volumeControl3 = volume3;


			// --> Vertical Position
			int verticalControl1 = (int)dato.pCtrl1;
			//verticalControl1 = posi1;

			int verticalControl2 = (int)dato.pCtrl2;
			//verticalControl2 = posi2;

			int verticalControl3 = (int)dato.pCtrl3;
			//verticalControl3 = posi3;


			// --> Rotation
			int rotationControl1 = (int)dato.rotCtrl1;
			//rotationControl1 = m_enc12;

			int rotationControl2 = (int)dato.rotCtrl2;
			//rotationControl2 = m_enc22;

			int rotationControl3 = (int)dato.rotCtrl3;
			//rotationControl3 = m_enc32;


			// --> Bend
			int bendControl1 = (int)dato.bCtrl1;
			//bendControl1 = m_enc11;

			int bendControl2 = (int)dato.bCtrl2;
			//bendControl2 = m_enc21;

			int bendControl3 = (int)dato.bCtrl3;
			//bendControl3 = m_enc31;


			// --> Conditions to check
			// Rigidity
			int rigidityCond123 = (int)dato.rigi123;
			//rigidityCond123 = key1;

			//key1 = key2;
			//key2 = key3;

			// Volume
			int volumeCond123 = (int)dato.vol123;
			//volumeCond123 = vc1f;

			//vc1f = vc2f;
			//vc2f = vc3f;

			// Vertical Movement
			//int verticalCond123 = (int)dato.vertical123;
			//verticalCond123 = pc1f;

			//pc1f = pc2f;
			//pc2f = pc3f;

			//Reduce rigidity
			int rigidityDown1 = (int)dato.down1;
			int rigidityDown2 = (int)dato.down2;
			int rigidityDown3 = (int)dato.down3;

			//Global Condition
			int globalCheck = (int)dato.globalCheck;


	   //-----------------------------------------------------------------------------------------------


		system("cls");
		start = timeGetTime();
		Pos.X = 0;
		Pos.Y = 0;
		SetConsoleCursorPosition(hOut, Pos);



		//if(_kbhit()){

			//switch(_getch()){


		//checkDown1 = rigidityDown1;

		//key1 = rigidityCond123;

		// _getch ---> Communicate with the serial port

		if (_getch){

			/*
			   ---> Insert here the control variables

			        They should be paired with the conditions like:

					i.e. rigidity
					point2 += controlRigidity2;
					key2 = 1;

					the conidtion, actually can be stated once since its the same for the entire series

			*/

			/*
			TRY:

			point = controlRigidity

			or

			point += controlRigidity

			or

			point -= controlRigidity

			*/

			if (globalCheck == 1){

				// Volume
				volume1 = volumeControl1;
				vc1f = volumeCond123;

				volume2 = volumeControl2;
				vc2f = vc1f;

				volume3 = volumeControl3;
				vc3f = vc2f;

				// Rigidity
				point1 = controlRigidity1;
				key1 = rigidityCond123;

				point2 = controlRigidity2;
				key2 = key1;

				point3 = controlRigidity3;
				key3 = key2;

				// Vertical
				posi1 = verticalControl1;
				pc1f = vc1f;
				posi2 = verticalControl2;
				pc2f = pc1f;
				posi3 = verticalControl3;
				pc3f = pc2f;


			}





			//key1 = rigidityCond123;

			//key2 = rigidityCond123;
			//key3 = key2;

			//point1 -= controlRigidity1;

			//point2 -= controlRigidity2;

			//point3 -= controlRigidity3;


			//m_enc12 += rotationControl1;

			//m_enc11 = m_enc11 - bendControl1;

			//pc2f = verticalCond123;

			//posi2 = verticalControl2;

			//checkDown1 = rigidityDown1;
			//checkDown2 = rigidityDown2;
			//checkDown3 = rigidityDown3;


			//key1 = rigidityCond123;

			//key2 = key1;
			//key3 = key2;

			/*
			vc1f = key3;
			vc2f = vc1f;
			vc3f = vc2f;

			pc1f = vc3f;
			pc2f = pc1f;
			pc3f = pc2f;

			// --> Rigidity Ctrl
			point1 = 0;
			point2 = 0;
			point3 = 0;

			// --> Volume Ctrl
			volume1 = 0;
			volume2 = 0;
			volume3 = 0;

			// --> Vertical Position
			//posi1 = 0;
			//posi2 = 0;
			//posi3 = 0;
			*/
			/*
			volume1 = 50;
			vc1f = 1;
			volume2 = 50;
			vc2f = 1;
			volume3 = 50;
			vc3f = 1;
			*/
			//vc1f = 1;
			//vc2f = 1;

			/*
			point1 = 10;
			key1 = 1;

			point2 = 10;
			key2 = 1;

			point3 = 10;
			key3 = 1;
			*/


			/*
			if (key1 == 1){


			    point1 = controlRigidity1;
				point2 = controlRigidity2;
				point3 = controlRigidity3;

				// --> Volume Ctrl
				volume1 += 40; //volumeControl1;
					volume2 += 40;//volumeControl2;
					volume3 += 40; //volumeControl3;

				// --> Vertical Position
                posi1 = verticalControl1;
				posi2 = verticalControl2;
				posi3 = verticalControl3;

				posi1 = verticalControl1;
				posi2 = verticalControl2;
				posi3 = verticalControl3;


			}*/








			// ---> write the value to actuator rigidity #2
			/*
			// UP
		 if (checkDown1 == 2 && key1 == 0){

				posi2 = posi2 + 10;

			}

			// DOWN
		 if (checkDown2 == 3 && !key1 == 0){

				posi2 = posi2 - 10;

			}*/

		// switch (key1) {  // checkUp
			/*
         // ---> Increase Volume
		 case(1):
			 volume1 += 10;
			 volume2 += 10;
		     volume3 += 10;
		     break;

		 case(2):
			 volume1 += 20;
			 volume2 += 20;
			 volume3 += 20;

		 case(3):
			 volume1 += 30;
			 volume2 += 30;
			 volume3 += 30;
			 */
			/*
		 //case(4) :
			if (key1 == 4) {
				volume1 += 40;
				volume2 += 40;
				volume3 += 40;
			}*/
			 /*
		 // ---> Reduce Volume
		 case(5) :
			 volume1 -= 10;
			 volume2 -= 10;
			 volume3 -= 10;
			 break;

		 case(6) :
			 volume1 -= 20;
			 volume2 -= 20;
			 volume3 -= 20;
			 break;

		 case(7) :
			 volume1 -= 30;
			 volume2 -= 30;
			 volume3 -= 30;
			 break;

		 case(8) :
			 volume1 -= 40;
			 volume2 -= 40;
			 volume3 -= 40;
			 break;

         // ---> Increase Rigidity

		 case(9) :
			 point1 += 1;
			 point1 += 1;
			 point1 += 1;
			 break;

		 case(10) :
			 point1 += 5;
			 point1 += 5;
			 point1 += 5;
			 break;

		 case(11) :
			 point1 += 10;
			 point1 += 10;
			 point1 += 10;
			 break;

		 case(12) :
			 point1 += 15;
			 point1 += 15;
			 point1 += 15;
			 break;

		 // ---> Reduce Rigidity

		 case(13) :
			 point1 -= 1;
			 point1 -= 1;
			 point1 -= 1;
			 break;

		 case(14) :
			 point1 -= 5;
			 point1 -= 5;
			 point1 -= 5;
			 break;

		 case(15) :
			 point1 -= 10;
			 point1 -= 10;
			 point1 -= 10;
			 break;

		 case(16) :
			 point1 -= 15;
			 point1 -= 15;
			 point1 -= 15;
			 break;

		 }*/


			/*
			//case '2': iVal2=50; break;
			//case '3': m_enc11 += 600; m_enc21 += 600; m_enc31 += 600; vc3f = 1; break;
			//case '1': Val_v1 += 600;  break;

			case '9': volume1 += 10; volume2 += 10; volume3 += 10; vc1f = 1; vc2f = 1; vc3f = 1; break;
			case '7': volume1 -= 10; volume2 -= 10; volume3 -= 10; vc1f = 1; vc2f = 1; vc3f = 1; break;
			//case '/': volume2 += 10; vc1f = 1; vc2f = 1; vc3f = 1; break;

			case '8': volume1 = 20; volume2 = 30; volume3 = 40; vc1f = 1; vc2f = 1; vc3f = 1; break;
			case 'a': volume1 = 40; volume2 = 40; volume3 = 40; vc1f = 1; vc2f = 1; vc3f = 1; break;

			case '6': posi1 = 30; posi2 = 20; posi3 = 10;  pc1f = 1; pc2f = 1; pc3f = 1;  break;
			case '4': posi1 = 10; posi2 = 10; posi3 = 10;  pc1f = 1; pc2f = 1; pc3f = 1;  break;


			//case '5': Val_p3 -= 100; pc3f = 1; break;
			//case '+': Val_p3 += 100; pc3f = 1; break;
			//case '@': Val_p3 -= 10; pc3f = 1; break;
			//case '[': Val_p3 += 10; pc3f = 1; break;

			case '3': point1 += 5; point2 += 5; point3 += 5; key1 = 1; key2 = 1; key3 = 1; break;
			case '1': point1 -= 5; point2 -= 5; point3 -= 5; key1 = 1; key2 = 1; key3 = 1; break;

			//case ':': point2 += 5; key1 = 1; key2 = 1; key3 = 1; break;
			//case ']': point2 = 2; key1 = 1; key2 = 1; key3 = 1; break;

			//case '3': Val_r1 += 1000; Val_r2 += 1000; Val_r3 += 1000; key1 = 1; key2 = 1; key3 = 1; break;
			//case '1': Val_r1 -= 1000; Val_r2 -= 1000; Val_r3 -= 1000; key1 = 1; key2 = 1; key3 = 1; break;
			case '2': point1 = 15; point2 = 10; point3 = 5; key1 = 1; key2 = 1; key3 = 1; break;
			//case 'd': point1 = 5; point2 = 10;  key1 = 1; key2 = 1; key3 = 1; break;
			case 'j': point1 = 2; point2 = 2;  point3 = 2; key1 = 1; key2 = 1; key3 = 1; break;
			case 'f': point1 = 9; point2 = 9;  point3 = 9; key1 = 1; key2 = 1; key3 = 1; break;

			case 'z': m_enc11 += 400; break;
			case 'x': m_enc11 -= 400; break;
			case 'c': m_enc21 += 400; break;
			case 'v': m_enc21 -= 400; break;
			case 'b': m_enc31 += 400; break;
			case 'n': m_enc31 -= 400; break;

			//case '-': Val_p2 += 100; break;
			//case '4': iVal =limit_max; break;
			//case '5': iVal =(limit_min+limit_max)/2; break;
			//case '6': iVal =limit_min; break;
			//case 'w': r_demo_frag = 1; break;
			//case 'e': p_demo_frag = 1; break;
			//case 'r': h_demo_frag = 1; pc1f = 1; pc2f = 1; pc3f = 1; break;
			//case 'b': p_demo_frag = 2; break;
			//case 't': v_demo_frag = 1; vc1f = 1; vc2f = 1; vc3f = 1; break;
			//case 'y': k_demo_frag = 1; key1 = 1; key2 = 1; key3 = 1; break;
			//case 'u': point1 -= 8; point2 -= 8; point3 -= 8; break;
			case 27: loop = FALSE; break;
			*/
			}
		//}

		if(iVal < limit_min)iVal=limit_min;
		if(iVal > limit_max)iVal=limit_max;
		if(iVal2 < 1)iVal2=1;
		if(iVal2 > 500)iVal2=500;

		//�f��
		//Posi_demo();
		//Rot_demo();
		High_demo();
		//Volume_demo();
		//Rigidity_demo();
		point1 = 5;
		//operate rigidity
		Rigidity_ctr1();
		Rigidity_ctr2();
		Rigidity_ctr3();
		//operate volume
		Volume_ctr1();
		Volume_ctr2();
		Volume_ctr3();
		//operate position
		Posi_ctr1();
		Posi_ctr2();
		Posi_ctr3();
		//operate rotation of tube
		Balloon_rot_ctr1();
		Balloon_rot_ctr2();
		Balloon_rot_ctr3();
		//operate benging of tube
		Balloon_bend_ctr1();
		Balloon_bend_ctr2();
		Balloon_bend_ctr3();

		if (Val_p1 < limit_min)Val_p1 = limit_min;
		if (Val_v1 > limit_max)Val_v1 = limit_max;
		if (Val_p2 < limit_min)Val_p2 = limit_min;
		if (Val_v2 > limit_max)Val_v2 = limit_max;
		if (Val_p3 < limit_min)Val_p3 = limit_min;
		if (Val_v3 > limit_max)Val_v3 = limit_max;

		SetData(&Module[0], Val_p1, Val_v1, Val_r1);
		SetData(&Module[1], Val_p2, Val_v2, Val_r2);
		SetData(&Module[2], Val_p3, Val_v3, Val_r3);
		SetData(&Module[3], motor12, motor11, 0);
		SetData(&Module[4], motor22, motor21, 0);
		SetData(&Module[5], motor32, motor31, 0);



		printf("%d %d %d\n", sen1, sen2, sen3); //pressure sensor
		printf("%d %d %d\n", point1, point2, point3);
		printf("atm %f %f %f\n", atm1, atm2, atm3);
		printf("%d %d %d\n", dig1, dig2, dig3);
		printf("%d %d %d %d %d %d\n", enc11, enc12, enc21, enc22, enc31, enc32); //encoder dc motors rotation
		printf("%d %d %d %d %d %d\n", m_enc11, m_enc12, m_enc21, m_enc22, m_enc31, m_enc32); //encoder dc motors bend
		printf("%d %d %d %d %d %d\n", posi1, posi2, posi3, volume1, volume2, volume3); //vertical position and volume (vertical inner tube)
		//LINEAR ACTUATORS
		printf("Val_v1 = %d Val_p1 = %d Val_r1 = %d\n", Val_v1, Val_p1, Val_r1); //current volume, current position, current rigidity
		printf("Val_v2 = %d Val_p2 = %d Val_r2 = %d\n", Val_v2, Val_p2, Val_r2);
		printf("Val_v3 = %d Val_p3 = %d Val_r3 = %d\n", Val_v3, Val_p3, Val_r3);
		//DC MOTORS
		printf("motor11 = %d motor12 = %d\n", motor11, motor12); //pwm bend | pwm rotate
		printf("motor21 = %d motor22 = %d\n", motor21, motor22);
		printf("motor31 = %d motor32 = %d\n", motor31, motor32);


		//test
		//std::cout << "data=" << dato.a << " " << dato.b << "        \n";

		/*
		//RECEIVED
		std::cout << "data = " << (int)dato.rCtrl1 << " " << (int)dato.rCtrl2 << " " << (int)dato.rCtrl3 << " "
			                   << (int)dato.vCtrl1 << " " << (int)dato.vCtrl2 << " " << (int)dato.vCtrl3 << " "
							   << (int)dato.pCtrl1 << " " << (int)dato.pCtrl2 << " " << (int)dato.pCtrl3 << " "
							   << (int)dato.rotCtrl1 << " " << (int)dato.rotCtrl2 << " " << (int)dato.rotCtrl3 << " "
				               << (int)dato.bCtrl1 << " " << (int)dato.bCtrl2 << " " << (int)dato.bCtrl3  << "        \n";
        */

		//Global condition
		std::cout << "Global condition = " << globalCheck << "        \n";

		//Conditions
		std::cout << "Volume condition = " << vc1f << " " << "Rigidity condition = " << key1 << "        \n";

		//Rigidity
		std::cout << "Internal pressure = " << sen1 << " " << sen2 << " " << sen3 << "        \n";
		//std::cout << "Values of rigidity = " << dig1 << " " << dig2 << " " << dig3 << "        \n";  //2
		//std::cout << "Condition for reducing rigidity = " << checkDown1 << " " << checkDown2 << " " << checkDown3 << "        \n";
		std::cout << "Actuator control rigidity = " << Val_r1 << " " << Val_r2 << " " << Val_r3<< "        \n";

		//Volume
		std::cout << "values of volume = " << vc1 << " " << vc2 << " " << vc3 << "        \n";
		std::cout << "Actuators Volume = " << Val_v1 << " " << Val_v2 << " " << Val_v3 << "        \n";

		//Vertical actuators
		std::cout << "Actuator vertical = " << Val_p1 << " " << Val_p2 << " " << Val_p3 << "        \n";
		std::cout << "Vertical ctrl = " << verticalControl1 << " " << verticalControl2 << " " << verticalControl3 << " " << "        \n";




		// �f�[�^�����M
		MES_GetTimeCount();

		for(i=0;i<CM_NUM-3;i++){
			if(!Module[i].Exchange(FALSE) ){ loop =FALSE;break;}
		}
		for (i = 3; i<CM_NUM; i++){
			if (!Module[i].Exchange2(FALSE)){ loop = FALSE; break; }
		}
		if(!Module[CM_NUM-1].Exchange(TRUE) ){ loop =FALSE;break;}

		MES_GetTimeCount();
		MES_GetTimeCount();
		Pos.X = 0;
		Pos.Y = 0;
		SetConsoleCursorPosition(hOut, Pos);
		stop = timeGetTime();

		/*
		//---------------------- receive osc ------------
		// the data to receive
		if (lis->isUpdated()) {
			MyDATA dato;
			//char data;
			lis->getData(dato);
			//std::cout << "data=" << data.a << " " << data.b << "        \n";
			printf("yes", dato.a, dato.b);
		}*/

		//--------------

		//--------------------------------- OSC SEND DATA --------------------------------------

		//these are just initialized locally
		char buffer[OUTPUT_BUFFER_SIZE];

		osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

		p << osc::BeginBundleImmediate   //???????????????????????

			//------- Test----------
			/*
			<< osc::BeginMessage("/test")
			<< sen1 << osc::EndMessage
			<< osc::BeginMessage("/test1")
			<< sen2 << osc::EndMessage
			<< osc::EndBundle;
			*/

			<< osc::BeginMessage("/pressureSens")
			//<< pre_sen1 << pre_sen2 << pre_sen3 << osc::EndMessage
			<< sen1 << sen2 << sen3 << osc::EndMessage
			<< osc::BeginMessage("/volUpdate")
			<< vc1 << vc2 << vc3 << osc::EndMessage
			<< osc::BeginMessage("/posUpdate")
			<< Val_p1_m << Val_p2_m << Val_p3_m << osc::EndMessage
			<< osc::BeginMessage("/rotUpdate")
			<< enc12 << enc22 << enc32 << osc::EndMessage
			<< osc::BeginMessage("/bendUpdate")
			<< enc11 << enc21 << enc31 << osc::EndMessage
			<< osc::EndBundle;


		socketTransmit->Send(p.Data(), p.Size());
		p.Clear();
		//------------------------------------

	}



	for(i=0;i<CM_NUM;i++){
		SetData( &Module[i], 0, 0, 0);
	}
	/*
	//---------------------- receive osc ------------
	// the data to receive
	if (lis->isUpdated()) {
		MyDATA data;
		//char data;
		lis->getData(data);
		//std::cout << "data=" << data.a << " " << data.b << "        \n";
		printf("data=", data.a, data.b);
	}
	//----------------------------
	*/

	for(i=0;i<CM_NUM-1;i++){
		if(!Module[i].Exchange(FALSE) ) loop =FALSE;
	}
	if(!Module[CM_NUM-1].Exchange(TRUE) ) loop =FALSE;

	/*
	//---------------- osc send ------------------
	//these are just initialized locally
	char buffer[OUTPUT_BUFFER_SIZE];

	osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

	p << osc::BeginBundleImmediate   //???????????????????????
		<< osc::BeginMessage("/test")
		<< val1 << osc::EndMessage
		<< osc::BeginMessage("/test1")
		<< val2 << osc::EndMessage
		<< osc::EndBundle;

	socketTransmit->Send(p.Data(), p.Size());
	//p.Clear();
	//------------------------------------
	*/

	/*while(0 != Module[0].stm_state.lPos){
		//Sleep(100);
		// �f�[�^�����M
		//		MES_GetTimeCount();
		for(i=0;i<CM_NUM-1;i++){
			if(!Module[i].Exchange(FALSE) ){ loop =FALSE;break;}
		}
		if(!Module[CM_NUM-1].Exchange(TRUE) ){ loop =FALSE;break;}
		//		MES_GetTimeCount();
		//for(i=0;i<CM_NUM;i++){
		//		IndecateData( &Module[i] );
		//}
	}*/
}

void PrintCM001Status( CM001* cm )
{
	printf("status ScModule %d CM001 %d\r\n", cm->pParentModule->bStatus, cm->bStatus );
}


//------------------------------------------------ MAIN --------------------------------------------------

int main( int argc, char* argv[] ){

	//-------------------------- OSC -------------------------------------

	//listener
	ExamplePacketListener listener;

	UdpListeningReceiveSocket s(
		IpEndpointName(IpEndpointName::ANY_ADDRESS, RECV_PORT),
		&listener);
	s1 = &s; //pointer

	//trasnmitter
	UdpTransmitSocket transmitSocket(IpEndpointName(ADDRESS, SEND_PORT));
	socketTransmit = &transmitSocket; //pointer

	std::cout << "press q to end\n";
	std::thread rv_th;
	rv_th = std::thread(recv_thread,&s);

	//value to send
	//int val1 = 2;
	//float val2 = 5.1;

	/*
	// ------------------- moved to EXCHANGE () -----------------------
	bool loop = true;

	while (loop) { // no infinite loop (in EXCHENGE)
		// the data to receive
		if (listener.isUpdated()) {
			MyDATA data;
			//char data;
			listener.getData(data);
			std::cout << "data=" << data.a << " " << data.b << "        \n";
		}
		//the data to send
        if (_kbhit()) { //don`t need maybe keyboard
			char key = _getch();
			if (key == 'q') { loop = false; break; }
			//--------------------- construct the message --------------------------
				p << osc::BeginBundleImmediate
					<< osc::BeginMessage("/test")
					<< key << val1 << osc::EndMessage
				    << osc::BeginMessage("/test1")
				    << key << val2 << osc::EndMessage
					<< osc::EndBundle;
				transmitSocket.Send(p.Data(), p.Size());
				p.Clear();


		}

	}
	//---> moved at the end
	s.AsynchronousBreak();

	if (rv_th.joinable()) {
		rv_th.join();
	}
	//return 0;
	*/

	//------------------------------------------------------------------------------------


	int i;
	// ������
	printf("Open ScModule ");
	if( ( ScModule.Open( 0xff ))==NULL ){
		printf( "Error %s\r\n", ScModule.Error );
		return( 0 );
	}
	printf("OK\r\n");

	for(i=0;i<CM_NUM;i++){
		printf("Open Module[%d] ",i);
		if( !Module[i].Open( &ScModule, 0, i )){
			printf( "%s", Module[i].Error );
			return( 0 );
		}
		printf("OK\r\n");
		PrintCM001Status( &Module[i]  );
	}

	// ���{�f�[�^�̑����M�iBasic�j
	Exchange(&listener);

	for(i=0;i<CM_NUM;i++){
		printf("Close Module[%d] ",i);
		if( !Module[i].Close() ){
			printf("Error:%s",Module[i].Error );
		}
		printf("OK\r\n");
	}
	/**/

	printf("ScModule Close ");
	if(!ScModule.Close()){
		printf("Error:%s",ScModule.Error );
	}
	printf("OK\r\n");


	//----- osc ------
	s.AsynchronousBreak();

	if (rv_th.joinable()) {
		rv_th.join();
	}
	//--------------------

	return(1);





}
