import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from esphome.const import CONF_ADDRESS, CONF_ID, CONF_UPDATE_INTERVAL, CONF_USE_ADDRESS



bus_t4_ns = cg.esphome_ns.namespace('bus_t4')
Nice = bus_t4_ns.class_('NiceBusT4', cover.Cover, cg.Component)

# Old pattern
#CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
#{
#    cv.GenerateID(): cv.declare_id(Nice),
#    cv.Optional(CONF_ADDRESS): cv.hex_uint16_t,
#    cv.Optional(CONF_USE_ADDRESS): cv.hex_uint16_t,
##    cv.Optional(CONF_UPDATE_INTERVAL): cv.positive_time_period_milliseconds,
#}
#).extend(cv.COMPONENT_SCHEMA)
#### example
#CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
#    {
#        cv.GenerateID(): cv.declare_id(RATGDOCover),
#        cv.Optional(CONF_ON_OPENING): automation.validate_automation(
#            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(CoverOpeningTrigger)}
#        ),
#        # ... other config options
#    }
#).extend(RATGDO_CLIENT_SCHEMA)
###

# New Pattern example
#CONFIG_SCHEMA = (
#    cover.cover_schema(RATGDOCover)  # Changed from COVER_SCHEMA to cover_schema()
#    .extend(
#        {
#            cv.Optional(CONF_ON_OPENING): automation.validate_automation(
#                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(CoverOpeningTrigger)}
#            ),
#            # ... other config options
#        }
#    )
#    .extend(RATGDO_CLIENT_SCHEMA)
#)

# New Pattern
CONFIG_SCHEMA = (
    cover.cover_schema(Nice)  # Changed from COVER_SCHEMA to cover_schema()
    .extend(
        {
#    cv.GenerateID(): cv.declare_id(Nice),
    cv.Optional(CONF_ADDRESS): cv.hex_uint16_t,
    cv.Optional(CONF_USE_ADDRESS): cv.hex_uint16_t,
#    cv.Optional(CONF_UPDATE_INTERVAL): cv.positive_time_period_milliseconds,
        }
            )
            .extend(cv.COMPONENT_SCHEMA)
)

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)

    yield cover.register_cover(var, config)

    if CONF_ADDRESS in config:
        address = config[CONF_ADDRESS]
        cg.add(var.set_to_address(address))

    if CONF_USE_ADDRESS in config:
        use_address = config[CONF_USE_ADDRESS]
        cg.add(var.set_from_address(use_address))
        
        
 #   if CONF_UPDATE_INTERVAL in config:
 #       update_interval = config[CONF_UPDATE_INTERVAL]
 #       cg.add(var.set_update_interval(update_interval))


