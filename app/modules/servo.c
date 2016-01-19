// Module for driving RC servos

#include "module.h"
#include "lauxlib.h"
#include "platform.h"

#include "c_types.h"
#include "c_string.h"

extern uint32_t system_get_time();

// Configuration and storage
#define MAX_NUM_SERVOS 8
#define MAX_SERVO_DELAY 3000
#define DELAY_SHIFT 10
static unsigned servo_pin[MAX_NUM_SERVOS];
static int servo_delay[MAX_NUM_SERVOS];
static int servo_min;
static int servo_stats;
static int servo_reduce_each_by_us;
static int servo_reduce_first_by_us;

//******************************************
// Initialization
static int lservo_init( lua_State* L )
{
	int i;
	for(i=0;i<MAX_NUM_SERVOS;i++){
		servo_pin[i] = 0;
		servo_delay[i] = 0;
	}
	servo_min=MAX_NUM_SERVOS;
	servo_stats=0;
	servo_reduce_each_by_us=2;
	servo_reduce_first_by_us=3;
  	return 0;
}
//******************************************
// Configuration
// 	Lua:	servo.config(stats,reduce_each_by_us,reduce_first_by_us)
static int lservo_config( lua_State* L )
{
	servo_stats=luaL_optinteger( L, 1 ,servo_stats);;
	servo_reduce_each_by_us=luaL_optinteger( L, 2 ,servo_reduce_each_by_us);
	servo_reduce_first_by_us=luaL_optinteger( L, 3 ,servo_reduce_first_by_us);
  	return 0;
}
//******************************************
// Set servo pulse length
// 	Lua:	servo.set(pin, us_pulse)
// 			us_delay=0 - disable servo pulse 
static int lservo_set( lua_State* L )
{
	int pin, delay;
	pin = luaL_checkinteger( L, 1 );
	MOD_CHECK_ID( gpio, pin );
	delay = luaL_checkinteger( L, 2 );
	if(delay!=0 && (delay<100 || delay>MAX_SERVO_DELAY) )
		return luaL_error( L, "wrong delay" );
	int i, ip=-1;
	// find and update this pin  	
	for(i=0;i<MAX_NUM_SERVOS;i++){
		if(servo_pin[i]==pin){
			ip=i;
			break;
		}
	}

	if(ip<0){
		// find and update unused pin
		for(i=0;i<MAX_NUM_SERVOS;i++){
			if(servo_delay[i]==0){
				ip=i;
				break;
			}
		}
	}
	if(ip<0){
		return luaL_error( L, "no more servo slots" );
	}
	// bubble down	
	for(;ip>0;ip--){
		i = ip-1;
		if(servo_delay[i]<=0) break;
		servo_pin[ip]	=servo_pin[i];
		servo_delay[ip]	=servo_delay[i];
	}
	servo_pin[ip]	=pin;
	servo_delay[ip]	=delay;
	// bubble up	
	for(;ip<(MAX_NUM_SERVOS-1);ip++){
		i = ip+1;
		if(servo_delay[i]>=delay){
			break;
		} else {
			servo_pin[ip]	=servo_pin[i];
			servo_delay[ip]	=servo_delay[i];
			servo_pin[i]	=pin;
			servo_delay[i]	=delay;
		}
	}
	for(servo_min=0;servo_min<MAX_NUM_SERVOS;servo_min++){
		if(servo_delay[servo_min]>0) break;	
	}
  	return 0;  
}
//******************************************
// Read stored pulse length
// Lua: servo.get(pin)
static int lservo_get( lua_State* L )
{
	int i, pin, delay=0;
	pin = luaL_checkinteger( L, 1 );
	MOD_CHECK_ID( gpio, pin );
	for(i=0;i<MAX_NUM_SERVOS;i++){
		if(servo_pin[i]==pin){
			delay=servo_delay[i];
			break;
		}
	}
	lua_pushinteger( L, delay );
  	return 1;
}
//******************************************
// Servo pulse on multiple pins
// call this from tmr about every 20ms
// Lua: servo.pulse(reduce_each_by_us,reduce_first_by_us)
static int lservo_pulse( lua_State* L )
{
	int i,d,d1;
	d1=servo_reduce_first_by_us;
	uint32_t tt[MAX_NUM_SERVOS],t1;
	for(i=servo_min;i<MAX_NUM_SERVOS;i++){
		noInterrupts();
		DIRECT_WRITE(servo_pin[i], HIGH);
		interrupts();
		//platform_gpio_write(servo_pin[i], 1);
		tt[i]=system_get_time();
		delayMicroseconds(DELAY_SHIFT);
	}
	for(i=servo_min;i<MAX_NUM_SERVOS;i++){
		t1=system_get_time();		
		d = servo_delay[i]+tt[i]-t1-servo_reduce_each_by_us-d1;
		d1=0;
		delayMicroseconds(d);
		noInterrupts();
		DIRECT_WRITE(servo_pin[i], LOW);
		interrupts();
		tt[i]-=system_get_time();
	}
	if(servo_stats){
		lua_newtable(L); 
		for(i=servo_min;i<MAX_NUM_SERVOS;i++){
			lua_pushinteger( L, servo_pin[i] );
			lua_pushinteger( L, tt[i] );
			lua_settable(L, -3);
		}
		return 1;
	} else return 0;
}

//*******************

// Module function map
static const LUA_REG_TYPE servo_map[] = {
  { LSTRKEY( "pulse" ), LFUNCVAL( lservo_pulse ) },
  { LSTRKEY( "set" ), LFUNCVAL( lservo_set ) },
  { LSTRKEY( "get" ), LFUNCVAL( lservo_get ) },
  { LSTRKEY( "init" ), LFUNCVAL( lservo_init ) },
  { LSTRKEY( "config" ), LFUNCVAL( lservo_config ) },
  { LNILKEY, LNILVAL }
};


NODEMCU_MODULE(SERVO, "servo", servo_map, lservo_init);
