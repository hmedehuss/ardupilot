#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "UARTDriver.h"
#include "Scheduler.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/select.h>
#include <string.h>
#include <fcntl.h>

#include <AP_Param/AP_Param.h>
#include <SITL/SIM_JSBSim.h>
#include <AP_HAL/utility/Socket.h>
#include "SITL/SIM_Aircraft.h"

#ifdef WIN32
	#include <winsock2.h>
#else
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <unistd.h> /* close */
	#include <netdb.h> /* gethostbyname */
	#define INVALID_SOCKET -1
	#define SOCKET_ERROR -1
	#define closesocket(s) close(s)
	typedef int SOCKET;
	typedef struct sockaddr_in SOCKADDR_IN;
	typedef struct sockaddr SOCKADDR;
	typedef struct in_addr IN_ADDR;
#endif

const char *destIPAdress = "192.168.0.202";
unsigned int inputPort = 51006;
unsigned int outputPort = 52006;
static SOCKET socket_input;
static SOCKADDR_IN addr_input;
static SOCKET socket_output;
static SOCKADDR_IN addr_output;

extern const AP_HAL::HAL& hal;

using namespace HALSITL;
using namespace SITL;

void SITL_State::_set_param_default(const char *parm)
{
    char *pdup = strdup(parm);
    char *p = strchr(pdup, '=');
    if (p == nullptr) {
        printf("Please specify parameter as NAME=VALUE");
        exit(1);
    }
    float value = strtof(p+1, nullptr);
    *p = 0;
    enum ap_var_type var_type;
    AP_Param *vp = AP_Param::find(pdup, &var_type);
    if (vp == nullptr) {
        printf("Unknown parameter %s\n", pdup);
        exit(1);
    }
    if (var_type == AP_PARAM_FLOAT) {
        ((AP_Float *)vp)->set_and_save(value);
    } else if (var_type == AP_PARAM_INT32) {
        ((AP_Int32 *)vp)->set_and_save(value);
    } else if (var_type == AP_PARAM_INT16) {
        ((AP_Int16 *)vp)->set_and_save(value);
    } else if (var_type == AP_PARAM_INT8) {
        ((AP_Int8 *)vp)->set_and_save(value);
    } else {
        printf("Unable to set parameter %s\n", pdup);
        exit(1);
    }
    printf("Set parameter %s to %f\n", pdup, value);
    free(pdup);
}


/*
  setup for SITL handling
 */
void SITL_State::_sitl_setup(const char *home_str)
{
    _home_str = home_str;

#if !defined(__CYGWIN__) && !defined(__CYGWIN64__)
    _parent_pid = getppid();
#endif

#ifndef HIL_MODE
    _setup_fdm();
#endif
    fprintf(stdout, "Starting SITL input\n");

    // find the barometer object if it exists
    _sitl = AP::sitl();
    _barometer = AP_Baro::get_singleton();
    _ins = AP_InertialSensor::get_singleton();
    _compass = Compass::get_singleton();

    if (_sitl != nullptr) {
        // setup some initial values
#ifndef HIL_MODE
        _update_airspeed(0);
        _update_gps(0, 0, 0, 0, 0, 0, 0, false);
        _update_rangefinder(0);
#endif
        if (enable_gimbal) {
            gimbal = new SITL::Gimbal(_sitl->state);
        }

        sitl_model->set_buzzer(&_sitl->buzzer_sim);
        sitl_model->set_sprayer(&_sitl->sprayer_sim);
        sitl_model->set_gripper_servo(&_sitl->gripper_sim);
        sitl_model->set_gripper_epm(&_sitl->gripper_epm_sim);
        sitl_model->set_parachute(&_sitl->parachute_sim);
        sitl_model->set_precland(&_sitl->precland_sim);
        _sitl->i2c_sim.init();
        sitl_model->set_i2c(&_sitl->i2c_sim);

        if (_use_fg_view) {
            fg_socket.connect(_fg_address, _fg_view_port);
        }

        fprintf(stdout, "Using Irlock at port : %d\n", _irlock_port);
        _sitl->irlock_port = _irlock_port;
    }

    _init_UPD_comm(destIPAdress, inputPort, outputPort);


    if (_synthetic_clock_mode) {
        // start with non-zero clock
        hal.scheduler->stop_clock(1);
    }
}


#ifndef HIL_MODE
/*
  setup a SITL FDM listening UDP port
 */
void SITL_State::_setup_fdm(void)
{
    if (!_sitl_rc_in.reuseaddress()) {
        fprintf(stderr, "SITL: socket reuseaddress failed on RC in port: %d - %s\n", _rcin_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    if (!_sitl_rc_in.bind("0.0.0.0", _rcin_port)) {
        fprintf(stderr, "SITL: socket bind failed on RC in port : %d - %s\n", _rcin_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    if (!_sitl_rc_in.set_blocking(false)) {
        fprintf(stderr, "SITL: socket set_blocking(false) failed on RC in port: %d - %s\n", _rcin_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    if (!_sitl_rc_in.set_cloexec()) {
        fprintf(stderr, "SITL: socket set_cloexec() failed on RC in port: %d - %s\n", _rcin_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
}
#endif


/*
  step the FDM by one time step
 */
void SITL_State::_fdm_input_step(void)
{
    static uint32_t last_pwm_input = 0;

    _fdm_input_local();

    /* make sure we die if our parent dies */
    if (kill(_parent_pid, 0) != 0) {
        exit(1);
    }

    if (_scheduler->interrupts_are_blocked() || _sitl == nullptr) {
        return;
    }

    // simulate RC input at 50Hz
    if (AP_HAL::millis() - last_pwm_input >= 20 && _sitl->rc_fail != SITL::SITL::SITL_RCFail_NoPulses) {
        last_pwm_input = AP_HAL::millis();
        new_rc_input = true;
    }

    _scheduler->sitl_begin_atomic();

    if (_update_count == 0 && _sitl != nullptr) {
        _update_gps(0, 0, 0, 0, 0, 0, 0, false);
        _scheduler->timer_event();
        _scheduler->sitl_end_atomic();
        return;
    }

    if (_sitl != nullptr) {
        _update_gps(_sitl->state.latitude, _sitl->state.longitude,
                    _sitl->state.altitude,
                    _sitl->state.speedN, _sitl->state.speedE, _sitl->state.speedD,
                    _sitl->state.yawDeg, true);
        _update_airspeed(_sitl->state.airspeed);
        _update_rangefinder(_sitl->state.range);

        if (_sitl->adsb_plane_count >= 0 &&
            adsb == nullptr) {
            adsb = new SITL::ADSB(_sitl->state, sitl_model->get_home());
        } else if (_sitl->adsb_plane_count == -1 &&
                   adsb != nullptr) {
            delete adsb;
            adsb = nullptr;
        }
    }

    // trigger all APM timers.
    _scheduler->timer_event();
    _scheduler->sitl_end_atomic();
}


void SITL_State::wait_clock(uint64_t wait_time_usec)
{
    while (AP_HAL::micros64() < wait_time_usec) {
        if (hal.scheduler->in_main_thread() ||
            Scheduler::from(hal.scheduler)->semaphore_wait_hack_required()) {
            _fdm_input_step();
        } else {
            usleep(1000);
        }
    }
    // check the outbound TCP queue size.  If it is too long then
    // MAVProxy/pymavlink take too long to process packets and it ends
    // up seeing traffic well into our past and hits time-out
    // conditions.
    if (sitl_model->get_speedup() > 1) {
        while (true) {
            const int queue_length = ((HALSITL::UARTDriver*)hal.uartA)->get_system_outqueue_length();
            // ::fprintf(stderr, "queue_length=%d\n", (signed)queue_length);
            if (queue_length < 1024) {
                break;
            }
            usleep(1000);
        }
    }
}

#define streq(a, b) (!strcmp(a, b))
int SITL_State::sim_fd(const char *name, const char *arg)
{
    if (streq(name, "vicon")) {
        if (vicon != nullptr) {
            AP_HAL::panic("Only one vicon system at a time");
        }
        vicon = new SITL::Vicon();
        return vicon->fd();
    } else if (streq(name, "benewake_tf02")) {
        if (benewake_tf02 != nullptr) {
            AP_HAL::panic("Only one benewake_tf02 at a time");
        }
        benewake_tf02 = new SITL::RF_Benewake_TF02();
        return benewake_tf02->fd();
    } else if (streq(name, "benewake_tf03")) {
        if (benewake_tf03 != nullptr) {
            AP_HAL::panic("Only one benewake_tf03 at a time");
        }
        benewake_tf03 = new SITL::RF_Benewake_TF03();
        return benewake_tf03->fd();
    } else if (streq(name, "benewake_tfmini")) {
        if (benewake_tfmini != nullptr) {
            AP_HAL::panic("Only one benewake_tfmini at a time");
        }
        benewake_tfmini = new SITL::RF_Benewake_TFmini();
        return benewake_tfmini->fd();
    } else if (streq(name, "lightwareserial")) {
        if (lightwareserial != nullptr) {
            AP_HAL::panic("Only one lightwareserial at a time");
        }
        lightwareserial = new SITL::RF_LightWareSerial();
        return lightwareserial->fd();
    } else if (streq(name, "lightwareserial-binary")) {
        if (lightwareserial_binary != nullptr) {
            AP_HAL::panic("Only one lightwareserial-binary at a time");
        }
        lightwareserial_binary = new SITL::RF_LightWareSerialBinary();
        return lightwareserial_binary->fd();
    } else if (streq(name, "lanbao")) {
        if (lanbao != nullptr) {
            AP_HAL::panic("Only one lanbao at a time");
        }
        lanbao = new SITL::RF_Lanbao();
        return lanbao->fd();
    } else if (streq(name, "blping")) {
        if (blping != nullptr) {
            AP_HAL::panic("Only one blping at a time");
        }
        blping = new SITL::RF_BLping();
        return blping->fd();
    } else if (streq(name, "leddarone")) {
        if (leddarone != nullptr) {
            AP_HAL::panic("Only one leddarone at a time");
        }
        leddarone = new SITL::RF_LeddarOne();
        return leddarone->fd();
    } else if (streq(name, "ulanding_v0")) {
        if (ulanding_v0 != nullptr) {
            AP_HAL::panic("Only one ulanding_v0 at a time");
        }
        ulanding_v0 = new SITL::RF_uLanding_v0();
        return ulanding_v0->fd();
    } else if (streq(name, "ulanding_v1")) {
        if (ulanding_v1 != nullptr) {
            AP_HAL::panic("Only one ulanding_v1 at a time");
        }
        ulanding_v1 = new SITL::RF_uLanding_v1();
        return ulanding_v1->fd();
    } else if (streq(name, "maxsonarseriallv")) {
        if (maxsonarseriallv != nullptr) {
            AP_HAL::panic("Only one maxsonarseriallv at a time");
        }
        maxsonarseriallv = new SITL::RF_MaxsonarSerialLV();
        return maxsonarseriallv->fd();
    } else if (streq(name, "wasp")) {
        if (wasp != nullptr) {
            AP_HAL::panic("Only one wasp at a time");
        }
        wasp = new SITL::RF_Wasp();
        return wasp->fd();
    } else if (streq(name, "nmea")) {
        if (nmea != nullptr) {
            AP_HAL::panic("Only one nmea at a time");
        }
        nmea = new SITL::RF_NMEA();
        return nmea->fd();

    } else if (streq(name, "rf_mavlink")) {
        if (wasp != nullptr) {
            AP_HAL::panic("Only one rf_mavlink at a time");
        }
        rf_mavlink = new SITL::RF_MAVLink();
        return rf_mavlink->fd();

    } else if (streq(name, "frsky-d")) {
        if (frsky_d != nullptr) {
            AP_HAL::panic("Only one frsky_d at a time");
        }
        frsky_d = new SITL::Frsky_D();
        return frsky_d->fd();
    // } else if (streq(name, "frsky-SPort")) {
    //     if (frsky_sport != nullptr) {
    //         AP_HAL::panic("Only one frsky_sport at a time");
    //     }
    //     frsky_sport = new SITL::Frsky_SPort();
    //     return frsky_sport->fd();

    // } else if (streq(name, "frsky-SPortPassthrough")) {
    //     if (frsky_sport_passthrough != nullptr) {
    //         AP_HAL::panic("Only one frsky_sport passthrough at a time");
    //     }
    //     frsky_sport = new SITL::Frsky_SPortPassthrough();
    //     return frsky_sportpassthrough->fd();
    } else if (streq(name, "crsf")) {
        if (crsf != nullptr) {
            AP_HAL::panic("Only one crsf at a time");
        }
        crsf = new SITL::CRSF();
        return crsf->fd();
    } else if (streq(name, "rplidara2")) {
        if (rplidara2 != nullptr) {
            AP_HAL::panic("Only one rplidara2 at a time");
        }
        rplidara2 = new SITL::PS_RPLidarA2();
        return rplidara2->fd();
    } else if (streq(name, "richenpower")) {
        sitl_model->set_richenpower(&_sitl->richenpower_sim);
        return _sitl->richenpower_sim.fd();
    } else if (streq(name, "gyus42v2")) {
        if (gyus42v2 != nullptr) {
            AP_HAL::panic("Only one gyus42v2 at a time");
        }
        gyus42v2 = new SITL::RF_GYUS42v2();
        return gyus42v2->fd();
    }

    AP_HAL::panic("unknown simulated device: %s", name);
}
int SITL_State::sim_fd_write(const char *name)
{
    if (streq(name, "vicon")) {
        if (vicon == nullptr) {
            AP_HAL::panic("No vicon created");
        }
        return vicon->write_fd();
    } else if (streq(name, "benewake_tf02")) {
        if (benewake_tf02 == nullptr) {
            AP_HAL::panic("No benewake_tf02 created");
        }
        return benewake_tf02->write_fd();
    } else if (streq(name, "benewake_tf03")) {
        if (benewake_tf03 == nullptr) {
            AP_HAL::panic("No benewake_tf03 created");
        }
        return benewake_tf03->write_fd();
    } else if (streq(name, "benewake_tfmini")) {
        if (benewake_tfmini == nullptr) {
            AP_HAL::panic("No benewake_tfmini created");
        }
        return benewake_tfmini->write_fd();
    } else if (streq(name, "lightwareserial")) {
        if (lightwareserial == nullptr) {
            AP_HAL::panic("No lightwareserial created");
        }
        return lightwareserial->write_fd();
    } else if (streq(name, "lightwareserial-binary")) {
        if (lightwareserial_binary == nullptr) {
            AP_HAL::panic("No lightwareserial_binary created");
        }
        return lightwareserial_binary->write_fd();
    } else if (streq(name, "lanbao")) {
        if (lanbao == nullptr) {
            AP_HAL::panic("No lanbao created");
        }
        return lanbao->write_fd();
    } else if (streq(name, "blping")) {
        if (blping == nullptr) {
            AP_HAL::panic("No blping created");
        }
        return blping->write_fd();
    } else if (streq(name, "leddarone")) {
        if (leddarone == nullptr) {
            AP_HAL::panic("No leddarone created");
        }
        return leddarone->write_fd();
    } else if (streq(name, "ulanding_v0")) {
        if (ulanding_v0 == nullptr) {
            AP_HAL::panic("No ulanding_v0 created");
        }
        return ulanding_v0->write_fd();
    } else if (streq(name, "ulanding_v1")) {
        if (ulanding_v1 == nullptr) {
            AP_HAL::panic("No ulanding_v1 created");
        }
        return ulanding_v1->write_fd();
    } else if (streq(name, "maxsonarseriallv")) {
        if (maxsonarseriallv == nullptr) {
            AP_HAL::panic("No maxsonarseriallv created");
        }
        return maxsonarseriallv->write_fd();
    } else if (streq(name, "wasp")) {
        if (wasp == nullptr) {
            AP_HAL::panic("No wasp created");
        }
        return wasp->write_fd();
    } else if (streq(name, "nmea")) {
        if (nmea == nullptr) {
            AP_HAL::panic("No nmea created");
        }
        return nmea->write_fd();
    } else if (streq(name, "rf_mavlink")) {
        if (rf_mavlink == nullptr) {
            AP_HAL::panic("No rf_mavlink created");
        }
        return rf_mavlink->write_fd();
    } else if (streq(name, "frsky-d")) {
        if (frsky_d == nullptr) {
            AP_HAL::panic("No frsky-d created");
        }
        return frsky_d->write_fd();
    } else if (streq(name, "crsf")) {
        if (crsf == nullptr) {
            AP_HAL::panic("No crsf created");
        }
        return crsf->write_fd();
    } else if (streq(name, "rplidara2")) {
        if (rplidara2 == nullptr) {
            AP_HAL::panic("No rplidara2 created");
        }
        return rplidara2->write_fd();
    } else if (streq(name, "richenpower")) {
        return _sitl->richenpower_sim.write_fd();
    } else if (streq(name, "gyus42v2")) {
        if (gyus42v2 == nullptr) {
            AP_HAL::panic("No gyus42v2 created");
        }
        return gyus42v2->write_fd();
    }
    AP_HAL::panic("unknown simulated device: %s", name);
}

#ifndef HIL_MODE
/*
  check for a SITL RC input packet
 */
void SITL_State::_check_rc_input(void)
{
    uint32_t count = 0;
    while (_read_rc_sitl_input()) {
        count++;
    }

    if (count > 100) {
        ::fprintf(stderr, "Read %u rc inputs\n", count);
    }
}

bool SITL_State::_read_rc_sitl_input()
{
    struct pwm_packet {
        uint16_t pwm[16];
    } pwm_pkt;

    const ssize_t size = _sitl_rc_in.recv(&pwm_pkt, sizeof(pwm_pkt), 0);
    switch (size) {
    case -1:
        return false;
    case 8*2:
    case 16*2: {
        // a packet giving the receiver PWM inputs
        for (uint8_t i=0; i<size/2; i++) {
            // setup the pwm input for the RC channel inputs
            if (i < _sitl->state.rcin_chan_count) {
                // we're using rc from simulator
                continue;
            }
            uint16_t pwm = pwm_pkt.pwm[i];
            if (pwm != 0) {
                if (_sitl->rc_fail == SITL::SITL::SITL_RCFail_Throttle950) {
                    if (i == 2) {
                        // set throttle (assumed to be on channel 3...)
                        pwm = 950;
                    } else {
                        // centre all other inputs
                        pwm = 1500;
                    }
                }
                pwm_input[i] = pwm;
            }
        }
        return true;
    }
    default:
        fprintf(stderr, "Malformed SITL RC input (%ld)", (long)size);
    }
    return false;
}

/*
  output current state to flightgear
 */
void SITL_State::_output_to_flightgear(void)
{
    SITL::FGNetFDM fdm {};
    const SITL::sitl_fdm &sfdm = _sitl->state;

    fdm.version = 0x18;
    fdm.padding = 0;
    fdm.longitude = DEG_TO_RAD_DOUBLE*sfdm.longitude;
    fdm.latitude = DEG_TO_RAD_DOUBLE*sfdm.latitude;
    fdm.altitude = sfdm.altitude;
    fdm.agl = sfdm.altitude;
    fdm.phi   = radians(sfdm.rollDeg);
    fdm.theta = radians(sfdm.pitchDeg);
    fdm.psi   = radians(sfdm.yawDeg);
    if (_vehicle == ArduCopter) {
        fdm.num_engines = 4;
        for (uint8_t i=0; i<4; i++) {
            fdm.rpm[i] = constrain_float((pwm_output[i]-1000), 0, 1000);
        }
    } else {
        fdm.num_engines = 4;
        fdm.rpm[0] = constrain_float((pwm_output[2]-1000)*3, 0, 3000);
        // for quadplane
        fdm.rpm[1] = constrain_float((pwm_output[5]-1000)*12, 0, 12000);
        fdm.rpm[2] = constrain_float((pwm_output[6]-1000)*12, 0, 12000);
        fdm.rpm[3] = constrain_float((pwm_output[7]-1000)*12, 0, 12000);
    }
    fdm.ByteSwap();

    fg_socket.send(&fdm, sizeof(fdm));
}

///////////////// UDP Comm section /////////////////
Vector3f SITL_State::speed_rot{0.0f, 0.0f, 0.0f};
Vector3f SITL_State::accel_body{0.0f, 0.0f, 0.0f};

int SITL_State::_init_UPD_comm(const char* IP_adress, unsigned int port_in, unsigned int port_out)
{
#ifdef WIN32
	WSADATA wsa;
	int err = WSAStartup(MAKEWORD(2, 2), &wsa);
	if(err < 0)
	{
		puts("WSAStartup failed !");
		exit(EXIT_FAILURE);
	}
#endif

	// Input initialisation
	socket_input = socket(AF_INET, SOCK_DGRAM, 0);
	if(socket_input == INVALID_SOCKET)
	{
		_sock_err_message("socket()");
		exit(_sock_error());
	}

	 // Put socket in nonblocking mode.
#ifdef WIN32
	unsigned long argp = 1;
	ioctlsocket(socket_input, FIONBIO, &argp);
#else
	fcntl(socket_input, F_SETFL, O_NDELAY);
#endif

	addr_input.sin_addr.s_addr = htonl(INADDR_ANY);
	addr_input.sin_family = AF_INET;
        addr_input.sin_port = htons(port_in);

	if(bind (socket_input, (SOCKADDR *) &addr_input, sizeof addr_input) == SOCKET_ERROR)
	{
		_sock_err_message("bind()");
		exit(_sock_error());
	}
	// Input initialized

	// Output initialisation
	socket_output = socket(AF_INET, SOCK_DGRAM, 0);
	if(socket_output == INVALID_SOCKET)
	{
		_sock_err_message("socket()");
		exit(_sock_error());
	}

	/* Put socket in nonblocking mode. */
#ifdef WIN32
	argp = 1;
	ioctlsocket(socket_output, FIONBIO, &argp);
#else
	fcntl(socket_output, F_SETFL, O_NDELAY);
#endif

        struct hostent *host_output = (struct hostent *) gethostbyname((char *)IP_adress);
	if (host_output == NULL)
	{
                fprintf (stderr, "Unknown host %s.\n", IP_adress);
		exit(EXIT_FAILURE);
	}

	addr_output.sin_family = AF_INET;
        addr_output.sin_port = htons(port_out);
        addr_output.sin_addr = *((struct in_addr *)host_output->h_addr);
	// Output initialized

	return 0;
}

void SITL_State::_UDP_comm_in(void){
	char recv_data[4096];
	char recv_data_tmp[4096];
	int bytes_read = 0;
	int bytes_read_tmp = 0;

#ifdef WIN32
	int addr_input_size = sizeof addr_input;
#else
	unsigned int addr_input_size = sizeof addr_input;
#endif
	do
	{
		memcpy(recv_data, recv_data_tmp, sizeof(recv_data));
		bytes_read = bytes_read_tmp;

		bytes_read_tmp = recvfrom(socket_input, recv_data_tmp, 4096, 0, (SOCKADDR *)&addr_input, &addr_input_size);
	}
	while(bytes_read_tmp != -1);

	recv_data[bytes_read] = '\0'; // to test if useful

	float latitude =0.0f;
	float longitude =0.0f;
	if(bytes_read > 0)
    {
		sscanf(recv_data, "%f %f %f %f %f %f %f %f ",
				&latitude,
				&longitude,
				&accel_body[0],
				&accel_body[1],
				&accel_body[2],
				&speed_rot[0],
				&speed_rot[1],
				&speed_rot[2]);
	}

	Aircraft::set_IMU_values(accel_body, speed_rot);
}

void SITL_State::_UDP_comm_out(void)
{
	// Cr\E9taion d'une structure 'fdm' dans laquelle copier les donn\E9es que l'ont veut extraire
	// Eventuellement, cr\E9ation d'une autre structure que la structure fdm existante, pour
	// avoir une structure sp\E9cifique \E0 notre besoin
	// Remplir la structure
	// creer une 'fg_socket' r\E9pondant \E0 notre besoin.

	//init_UPD_comm(destIPAdress, inputPort, outputPort);
	char buffer[4096];
	memset(&buffer, 0, sizeof(buffer));
    sprintf(buffer, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ",
           (static_cast<float>(pwm_output[6])-1000.0)/1000.0, // i.e. LH_FRONT_TOP_ONE
           (static_cast<float>(pwm_output[6])-1000.0)/1000.0, // i.e. LH_FRONT_TOP_TWO
           (static_cast<float>(pwm_output[9])-1000.0)/1000.0, // i.e. LH_FRONT_DOWN_ONE
           (static_cast<float>(pwm_output[9])-1000.0)/1000.0, // i.e. LH_FRONT_DOWN_TWO
           (static_cast<float>(pwm_output[5])-1000.0)/1000.0, // i.e. RH_FRONT_TOP_ONE
           (static_cast<float>(pwm_output[5])-1000.0)/1000.0, // i.e. RH_FRONT_TOP_TWO
           (static_cast<float>(pwm_output[10])-1000.0)/1000.0, // i.e. RH_FRONT_DOWN_ONE
           (static_cast<float>(pwm_output[10])-1000.0)/1000.0, // i.e. RH_FRONT_DOWN_TWO
           (static_cast<float>(pwm_output[7])-1000.0)/1000.0, // i.e. LH_REAR_TOP_ONE
           (static_cast<float>(pwm_output[7])-1000.0)/1000.0, // i.e. LH_REAR_TOP_TWO
           (static_cast<float>(pwm_output[12])-1000.0)/1000.0, // i.e. LH_REAR_DOWN_ONE
           (static_cast<float>(pwm_output[12])-1000.0)/1000.0, // i.e. LH_REAR_DOWN_TWO
           (static_cast<float>(pwm_output[8])-1000.0)/1000.0, // i.e. RH_REAR_TOP_ONE
           (static_cast<float>(pwm_output[8])-1000.0)/1000.0, // i.e. RH_REAR_TOP_TWO
           (static_cast<float>(pwm_output[11])-1000.0)/1000.0,// i.e. RH_REAR_DOWN_ONE
           (static_cast<float>(pwm_output[11])-1000.0)/1000.0,// i.e. RH_REAR_DOWN_TWO
    	   (static_cast<float>(pwm_output[3])-1000.0)/1000.0,// i.e. LH_PUSHER
		   (static_cast<float>(pwm_output[13])-1000.0)/1000.0);// i.e. RH_PUSHER

    if(sendto(socket_output, buffer, static_cast<int>(strlen(buffer)), 0, (SOCKADDR *)&addr_output, sizeof addr_output) < 0)
	{
		_sock_err_message("sendto()");
		exit(_sock_error());
	}
}

int SITL_State::_sock_error(void)
{
#ifdef WIN32
	return WSAGetLastError();
#else
	return errno;
  #endif
}

void SITL_State::_sock_err_message(const char* msg)
{
#ifdef WIN32
	fprintf (stderr, "Error : %s.\n", msg);
	exit(EXIT_FAILURE);
#else
	perror(msg);
#endif
}

void SITL_State::_close_UDP_comm(void){
	closesocket(socket_input);
#ifdef WIN32
	WSACleanup();
#endif
}
///////////////// end UDP Comm section /////////////////

/*
  get FDM input from a local model
 */
void SITL_State::_fdm_input_local(void)
{
    struct sitl_input input;

    // check for direct RC input
    _check_rc_input();

    // construct servos structure for FDM
    _simulator_servos(input);

    // update the model
    sitl_model->update_model(input);

    // get FDM output from the model
    if (_sitl) {
        sitl_model->fill_fdm(_sitl->state);
        _sitl->update_rate_hz = sitl_model->get_rate_hz();

        if (_sitl->rc_fail == SITL::SITL::SITL_RCFail_None) {
            for (uint8_t i=0; i< _sitl->state.rcin_chan_count; i++) {
                pwm_input[i] = 1000 + _sitl->state.rcin[i]*1000;
            }
        }
    }

    if (gimbal != nullptr) {
        gimbal->update();
    }
    if (adsb != nullptr) {
        adsb->update();
    }
    if (vicon != nullptr) {
        Quaternion attitude;
        sitl_model->get_attitude(attitude);
        vicon->update(sitl_model->get_location(),
                      sitl_model->get_position(),
                      sitl_model->get_velocity_ef(),
                      attitude);
    }
    if (benewake_tf02 != nullptr) {
        benewake_tf02->update(sitl_model->rangefinder_range());
    }
    if (benewake_tf03 != nullptr) {
        benewake_tf03->update(sitl_model->rangefinder_range());
    }
    if (benewake_tfmini != nullptr) {
        benewake_tfmini->update(sitl_model->rangefinder_range());
    }
    if (lightwareserial != nullptr) {
        lightwareserial->update(sitl_model->rangefinder_range());
    }
    if (lightwareserial_binary != nullptr) {
        lightwareserial_binary->update(sitl_model->rangefinder_range());
    }
    if (lanbao != nullptr) {
        lanbao->update(sitl_model->rangefinder_range());
    }
    if (blping != nullptr) {
        blping->update(sitl_model->rangefinder_range());
    }
    if (leddarone != nullptr) {
        leddarone->update(sitl_model->rangefinder_range());
    }
    if (ulanding_v0 != nullptr) {
        ulanding_v0->update(sitl_model->rangefinder_range());
    }
    if (ulanding_v1 != nullptr) {
        ulanding_v1->update(sitl_model->rangefinder_range());
    }
    if (maxsonarseriallv != nullptr) {
        maxsonarseriallv->update(sitl_model->rangefinder_range());
    }
    if (wasp != nullptr) {
        wasp->update(sitl_model->rangefinder_range());
    }
    if (nmea != nullptr) {
        nmea->update(sitl_model->rangefinder_range());
    }
    if (rf_mavlink != nullptr) {
        rf_mavlink->update(sitl_model->rangefinder_range());
    }
    if (gyus42v2 != nullptr) {
        gyus42v2->update(sitl_model->rangefinder_range());
    }

    if (frsky_d != nullptr) {
        frsky_d->update();
    }
    // if (frsky_sport != nullptr) {
    //     frsky_sport->update();
    // }
    // if (frsky_sportpassthrough != nullptr) {
    //     frsky_sportpassthrough->update();
    // }

    if (crsf != nullptr) {
        crsf->update();
    }

    if (rplidara2 != nullptr) {
        rplidara2->update(sitl_model->get_location());
    }

    if (_sitl) {
        _sitl->efi_ms.update();
    }

    if (_sitl && _use_fg_view) {
        _output_to_flightgear();
    }

    _UDP_comm_out();
    _UDP_comm_in();

    // update simulation time
    if (_sitl) {
        hal.scheduler->stop_clock(_sitl->state.timestamp_us);
    } else {
        hal.scheduler->stop_clock(AP_HAL::micros64()+100);
    }

    set_height_agl();

    _synthetic_clock_mode = true;
    _update_count++;
}
#endif

/*
  create sitl_input structure for sending to FDM
 */
void SITL_State::_simulator_servos(struct sitl_input &input)
{
    static uint32_t last_update_usec;

    /* this maps the registers used for PWM outputs. The RC
     * driver updates these whenever it wants the channel output
     * to change */
    uint8_t i;

    if (last_update_usec == 0 || !output_ready) {
        for (i=0; i<SITL_NUM_CHANNELS; i++) {
            pwm_output[i] = 1000;
        }
        if (_vehicle == ArduPlane) {
            pwm_output[0] = pwm_output[1] = pwm_output[3] = 1500;
        }
        if (_vehicle == Rover) {
            pwm_output[0] = pwm_output[1] = pwm_output[2] = pwm_output[3] = 1500;
        }
    }

    // output at chosen framerate
    uint32_t now = AP_HAL::micros();
    last_update_usec = now;

    float altitude = _barometer?_barometer->get_altitude():0;
    float wind_speed = 0;
    float wind_direction = 0;
    float wind_dir_z = 0;

    // give 5 seconds to calibrate airspeed sensor at 0 wind speed
    if (wind_start_delay_micros == 0) {
        wind_start_delay_micros = now;
    } else if (_sitl && (now - wind_start_delay_micros) > 5000000 ) {
        // The EKF does not like step inputs so this LPF keeps it happy.
        wind_speed =     _sitl->wind_speed_active     = (0.95f*_sitl->wind_speed_active)     + (0.05f*_sitl->wind_speed);
        wind_direction = _sitl->wind_direction_active = (0.95f*_sitl->wind_direction_active) + (0.05f*_sitl->wind_direction);
        wind_dir_z =     _sitl->wind_dir_z_active     = (0.95f*_sitl->wind_dir_z_active)     + (0.05f*_sitl->wind_dir_z);
        
        // pass wind into simulators using different wind types via param SIM_WIND_T*.
        switch (_sitl->wind_type) {
        case SITL::SITL::WIND_TYPE_SQRT:
            if (altitude < _sitl->wind_type_alt) {
                wind_speed *= sqrtf(MAX(altitude / _sitl->wind_type_alt, 0));
            }
            break;

        case SITL::SITL::WIND_TYPE_COEF:
            wind_speed += (altitude - _sitl->wind_type_alt) * _sitl->wind_type_coef;
            break;

        case SITL::SITL::WIND_TYPE_NO_LIMIT:
        default:
            break;
        }

        // never allow negative wind velocity
        wind_speed = MAX(wind_speed, 0);
    }

    input.wind.speed = wind_speed;
    input.wind.direction = wind_direction;
    input.wind.turbulence = _sitl?_sitl->wind_turbulance:0;
    input.wind.dir_z = wind_dir_z;

    for (i=0; i<SITL_NUM_CHANNELS; i++) {
        if (pwm_output[i] == 0xFFFF) {
            input.servos[i] = 0;
        } else {
            input.servos[i] = pwm_output[i];
        }
    }

    float engine_mul = _sitl?_sitl->engine_mul.get():1;
    uint8_t engine_fail = _sitl?_sitl->engine_fail.get():0;
    float throttle = 0.0f;
    
    if (engine_fail >= ARRAY_SIZE(input.servos)) {
        engine_fail = 0;
    }
    // apply engine multiplier to motor defined by the SIM_ENGINE_FAIL parameter
    if (_vehicle != Rover) {
        input.servos[engine_fail] = ((input.servos[engine_fail]-1000) * engine_mul) + 1000;
    } else {
        input.servos[engine_fail] = static_cast<uint16_t>(((input.servos[engine_fail] - 1500) * engine_mul) + 1500);
    }

    if (_vehicle == ArduPlane) {
        float forward_throttle = constrain_float((input.servos[2] - 1000) / 1000.0f, 0.0f, 1.0f);
        // do a little quadplane dance
        float hover_throttle = 0.0f;
        uint8_t running_motors = 0;
        for (i=0; i < sitl_model->get_num_motors() - 1; i++) {
            float motor_throttle = constrain_float((input.servos[sitl_model->get_motors_offset() + i] - 1000) / 1000.0f, 0.0f, 1.0f);
            // update motor_on flag
            if (!is_zero(motor_throttle)) {
                hover_throttle += motor_throttle;
                running_motors++;
            }
        }
        if (running_motors > 0) {
            hover_throttle /= running_motors;
        }
        if (!is_zero(forward_throttle)) {
            throttle = forward_throttle;
        } else {
            throttle = hover_throttle;
        }
    } else if (_vehicle == Rover) {
        input.servos[2] = static_cast<uint16_t>(constrain_int16(input.servos[2], 1000, 2000));
        input.servos[0] = static_cast<uint16_t>(constrain_int16(input.servos[0], 1000, 2000));
        throttle = fabsf((input.servos[2] - 1500) / 500.0f);
    } else {
        // run checks on each motor
        uint8_t running_motors = 0;
        for (i=0; i < sitl_model->get_num_motors(); i++) {
            float motor_throttle = constrain_float((input.servos[i] - 1000) / 1000.0f, 0.0f, 1.0f);
            // update motor_on flag
            if (!is_zero(motor_throttle)) {
                throttle += motor_throttle;
                running_motors++;
            }
        }
        if (running_motors > 0) {
            throttle /= running_motors;
        }
    }
    if (_sitl) {
        _sitl->throttle = throttle;
    }

    float voltage = 0;
    _current = 0;
    
    if (_sitl != nullptr) {
        if (_sitl->state.battery_voltage <= 0) {
            if (_vehicle == ArduSub) {
                voltage = _sitl->batt_voltage;
                for (i = 0; i < 6; i++) {
                    float pwm = input.servos[i];
                    //printf("i: %d, pwm: %.2f\n", i, pwm);
                    float fraction = fabsf((pwm - 1500) / 500.0f);

                    voltage -= fraction * 0.5f;

                    float draw = fraction * 15;
                    _current += draw;
                }
            } else {
                // simulate simple battery setup
                // lose 0.7V at full throttle
                voltage = _sitl->batt_voltage - 0.7f * throttle;

                // assume 50A at full throttle
                _current = 50.0f * throttle;
            }
        } else {
            // FDM provides voltage and current
            voltage = _sitl->state.battery_voltage;
            _current = _sitl->state.battery_current;
        }
    }

    // assume 3DR power brick
    voltage_pin_value = ((voltage / 10.1f) / 5.0f) * 1024;
    current_pin_value = ((_current / 17.0f) / 5.0f) * 1024;
    // fake battery2 as just a 25% gain on the first one
    voltage2_pin_value = ((voltage * 0.25f / 10.1f) / 5.0f) * 1024;
    current2_pin_value = ((_current * 0.25f / 17.0f) / 5.0f) * 1024;
}

void SITL_State::init(int argc, char * const argv[])
{
    pwm_input[0] = pwm_input[1] = pwm_input[3] = 1500;
    pwm_input[4] = pwm_input[7] = 1800;
    pwm_input[2] = pwm_input[5] = pwm_input[6] = 1000;

    _scheduler = Scheduler::from(hal.scheduler);
    _parse_command_line(argc, argv);
}

/*
  set height above the ground in meters
 */
void SITL_State::set_height_agl(void)
{
    static float home_alt = -1;

    if (!_sitl) {
        // in example program
        return;
    }

    if (is_equal(home_alt, -1.0f) && _sitl->state.altitude > 0) {
        // remember home altitude as first non-zero altitude
        home_alt = _sitl->state.altitude;
    }

#if AP_TERRAIN_AVAILABLE
    if (_sitl != nullptr &&
        _sitl->terrain_enable) {
        // get height above terrain from AP_Terrain. This assumes
        // AP_Terrain is working
        float terrain_height_amsl;
        struct Location location;
        location.lat = _sitl->state.latitude*1.0e7;
        location.lng = _sitl->state.longitude*1.0e7;

        AP_Terrain *_terrain = AP_Terrain::get_singleton();
        if (_terrain != nullptr &&
            _terrain->height_amsl(location, terrain_height_amsl, false)) {
            _sitl->height_agl = _sitl->state.altitude - terrain_height_amsl;
            return;
        }
    }
#endif

    if (_sitl != nullptr) {
        // fall back to flat earth model
        _sitl->height_agl = _sitl->state.altitude - home_alt;
    }
}

#endif
