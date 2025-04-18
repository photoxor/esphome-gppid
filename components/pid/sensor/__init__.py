import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    ICON_GAUGE,
    CONF_TYPE,
)

from .. import pid_ns, PIDComponent

PIDSensor = pid_ns.class_("PIDSensor", sensor.Sensor, cg.Component)
PIDSensorType = pid_ns.enum("PIDSensorType")

PID_SENSOR_TYPES = {
    "RESULT": PIDSensorType.PID_SENSOR_TYPE_RESULT,
    "ERROR": PIDSensorType.PID_SENSOR_TYPE_ERROR,
    "PROPORTIONAL": PIDSensorType.PID_SENSOR_TYPE_PROPORTIONAL,
    "INTEGRAL": PIDSensorType.PID_SENSOR_TYPE_INTEGRAL,
    "DERIVATIVE": PIDSensorType.PID_SENSOR_TYPE_DERIVATIVE,
    "KP": PIDSensorType.PID_SENSOR_TYPE_KP,
    "KI": PIDSensorType.PID_SENSOR_TYPE_KI,
    "KD": PIDSensorType.PID_SENSOR_TYPE_KD,
}

CONF_PID_ID = "pid_id"
CONFIG_SCHEMA = (
    sensor.sensor_schema(
        PIDSensor,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.GenerateID(CONF_PID_ID): cv.use_id(PIDComponent),
            cv.Required(CONF_TYPE): cv.enum(PID_SENSOR_TYPES, upper=True),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_PID_ID])
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    cg.add(var.set_parent(parent))
    cg.add(var.set_type(config[CONF_TYPE]))
