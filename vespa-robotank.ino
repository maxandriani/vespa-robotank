#include "RoboCore_Vespa.h"
#include "motor.h"
#include "vespa-robotank.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <string.h>
#include <BLE2902.h>

static const BLEUUID BATTERY_SERVICE_UUID((uint16_t) 0x180F); // org.bluetooth.service.battery_service
static const BLEUUID BATTERY_LEVEL_GATT_UUID((uint16_t) 0x2A19); // org.bluetooth.characteristic.battery_level

static const BLEUUID ROBOT_SERVICE_UUID("eab5cc75-9748-4759-bcf8-9d0b11735de1");
static const BLEUUID ROBOT_LEFT_MOTOR_AXIS_GATT_UUID("33f6c513-bd94-407f-bc7c-0260551a605e");
static const BLEUUID ROBOT_RIGHT_MOTOR_AXIS_GATT_UUID("4b6a7037-3a87-47b6-b266-bc907c34f150");

static const BLEUUID ARM_SERVICE_UUID("8a342fce-09ed-4c62-9380-2703b9a58e2e");
static const BLEUUID HORIZONTAL_ARM_AXIS_GATT_UUID("a1982d1b-4cc7-4f0b-aa46-c199f9531a26");
static const BLEUUID VERTICAL_ARM_AXIS_GATT_UUID("f6a7de4b-1d42-4bb5-a48a-ef839645ede4");
static const BLEUUID DISTANCE_ARM_AXIS_GATT_UUID("3d5c0ebe-8d3c-434a-86e6-827abe0207be");
static const BLEUUID CLAW_ARM_AXIS_GATT_UUID("c54c774d-d37b-4857-935f-149245483f5f");

static Motor left(LEFT_MOTOR_FORWARD,
                  LEFT_MOTOR_BACKWARD,
                  LEFT_MOTOR_CHANNEL,
                  MOTOR_DEFAULT_POWER);

static Motor right(RIGHT_MOTOR_FORWARD,
                   RIGHT_MOTOR_BACKWARD,
                   RIGHT_MOTOR_CHANNEL,
                   MOTOR_DEFAULT_POWER);

static VespaServo horizontal;
static VespaServo vertical;
static VespaServo distance;
static VespaServo claw;

static VespaBattery battery;
static uint8_t battery_capacity = 0;
static bool client_connected = false;

BLEServer *pServer;
BLEService *pRobotService;
BLEService *pArmService;
BLEService *pBatteryService;
BLECharacteristic *pLeftMotorAxisChar;
BLECharacteristic *pRightMotorAxisChar;
BLECharacteristic *pHorizontalArmAxisChar;
BLECharacteristic *pVerticalArmAxisChar;
BLECharacteristic *pDistanceArmAxisChar;
BLECharacteristic *pClawArmAxisChar;
BLECharacteristic *pBuzzChar;
BLECharacteristic *pBatteryLevelChar;

class MotorAxisCallbacks: public BLECharacteristicCallbacks {
    public:
        MotorAxisCallbacks(Motor *motor):
            _motor(motor)
        {
        }

        void onWrite(BLECharacteristic *pDescriptor, esp_ble_gatts_cb_param_t* param) {
            int8_t axis;
            memcpy(&axis, pDescriptor->getData(), sizeof(int8_t));
            if (axis > 100)
                axis = 100;
            if (axis < -100)
                axis = -100;

            Serial.print("Velocity ");
            Serial.println(axis);
            _motor->velocity(axis);
        }

    private:
        Motor * _motor;
};

class ServoAxisCallbacks: public BLECharacteristicCallbacks {
    public:
        ServoAxisCallbacks(VespaServo *servo, uint8_t min, uint8_t max, String name) :
            _servo(servo),
            _min_range(min),
            _max_range(max),
            _name(name)
        {}

        void onWrite(BLECharacteristic *pDescriptor, esp_ble_gatts_cb_param_t* param) {
            uint8_t axis;
            memcpy(&axis, pDescriptor->getData(), sizeof(uint8_t));
            if (axis > _max_range)
                axis = _max_range;
            if (axis < _min_range)
                axis = _min_range;

            Serial.print("Servo ");
            Serial.print(_name);
            Serial.print(": ");
            Serial.println(axis);
            _servo->write(axis);
        }

    private:
        VespaServo *_servo;
        String _name;
        uint8_t _min_range;
        uint8_t _max_range;
};

class RobotBleServerCallbacks : public BLEServerCallbacks
{
    public:
        void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param){
            client_connected = true;
            Serial.println("Client connected");
        }

        void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
            client_connected = false;
            Serial.println("Client disconnected");
        }
};

void setup_battery_service() {
    pBatteryService = pServer->createService(BATTERY_SERVICE_UUID);
    pBatteryLevelChar = pBatteryService->createCharacteristic(BATTERY_LEVEL_GATT_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pBatteryLevelChar->addDescriptor(new BLE2902());
    pBatteryService->start();
}

void setup_robot_service() {
    pRobotService = pServer->createService(ROBOT_SERVICE_UUID);
    pLeftMotorAxisChar = pRobotService->createCharacteristic(ROBOT_LEFT_MOTOR_AXIS_GATT_UUID, BLECharacteristic::PROPERTY_WRITE);
    pRightMotorAxisChar = pRobotService->createCharacteristic(ROBOT_RIGHT_MOTOR_AXIS_GATT_UUID, BLECharacteristic::PROPERTY_WRITE);

    BLEDescriptor *validRangeL = new BLEDescriptor((uint16_t)0x2906); // org.bluetooth.descriptor.valid_range
    BLEDescriptor *validRangeR = new BLEDescriptor((uint16_t)0x2906); // org.bluetooth.descriptor.valid_range
    int8_t range[] = {-100, 100};
    validRangeL->setValue((uint8_t *) range, sizeof(range));
    validRangeR->setValue((uint8_t *) range, sizeof(range));
    BLEDescriptor *leftDesc = new BLEDescriptor((uint16_t)0x2901); // org.bluetooth.descriptor.gatt.characteristic_user_description
    leftDesc->setValue("Left Motor");
    pLeftMotorAxisChar->addDescriptor(leftDesc);
    pLeftMotorAxisChar->addDescriptor(validRangeL);
    pLeftMotorAxisChar->setCallbacks(new MotorAxisCallbacks(&left));

    BLEDescriptor *rightDesc = new BLEDescriptor((uint16_t)0x2901); // org.bluetooth.descriptor.gatt.characteristic_user_description
    rightDesc->setValue("Right Motor");
    pRightMotorAxisChar->addDescriptor(rightDesc);
    pRightMotorAxisChar->addDescriptor(validRangeR);
    pRightMotorAxisChar->setCallbacks(new MotorAxisCallbacks(&right));

    pRobotService->start();
}

void setup_arm_service() {
    pArmService = pServer->createService(ARM_SERVICE_UUID);
    pHorizontalArmAxisChar = pArmService->createCharacteristic(HORIZONTAL_ARM_AXIS_GATT_UUID, BLECharacteristic::PROPERTY_WRITE);
    pVerticalArmAxisChar = pArmService->createCharacteristic(VERTICAL_ARM_AXIS_GATT_UUID, BLECharacteristic::PROPERTY_WRITE);
    pDistanceArmAxisChar = pArmService->createCharacteristic(DISTANCE_ARM_AXIS_GATT_UUID, BLECharacteristic::PROPERTY_WRITE);
    pClawArmAxisChar = pArmService->createCharacteristic(CLAW_ARM_AXIS_GATT_UUID, BLECharacteristic::PROPERTY_WRITE);

    BLEDescriptor *horizontalRange = new BLEDescriptor((uint16_t)0x2906); // org.bluetooth.descriptor.valid_range
    uint8_t vHorizontalRange[] = {0, 180};
    horizontalRange->setValue((uint8_t *) vHorizontalRange, sizeof(vHorizontalRange));

    BLEDescriptor *verticalRange = new BLEDescriptor((uint16_t)0x2906); // org.bluetooth.descriptor.valid_range
    uint8_t vVerticalRange[] = {0, 180};
    verticalRange->setValue((uint8_t *)vVerticalRange, sizeof(vVerticalRange));

    BLEDescriptor *distanceRange = new BLEDescriptor((uint16_t)0x2906); // org.bluetooth.descriptor.valid_range
    uint8_t vDistanceRange[] = {0, 180};
    distanceRange->setValue((uint8_t *)vDistanceRange, sizeof(vDistanceRange));

    BLEDescriptor *clawRange = new BLEDescriptor((uint16_t)0x2906); // org.bluetooth.descriptor.valid_range
    uint8_t vClawRange[] = {0, 180};
    clawRange->setValue((uint8_t *)vClawRange, sizeof(vClawRange));

    BLEDescriptor *horizontalDesc = new BLEDescriptor((uint16_t)0x2901); // org.bluetooth.descriptor.gatt.characteristic_user_description
    horizontalDesc->setValue("Horizontal Arm");
    
    BLEDescriptor *verticalDesc = new BLEDescriptor((uint16_t)0x2901); // org.bluetooth.descriptor.gatt.characteristic_user_description
    verticalDesc->setValue("Vertical Arm");
    
    BLEDescriptor *distanceDesc = new BLEDescriptor((uint16_t)0x2901); // org.bluetooth.descriptor.gatt.characteristic_user_description
    distanceDesc->setValue("Distance ARM");
    
    BLEDescriptor *clawDesc = new BLEDescriptor((uint16_t)0x2901); // org.bluetooth.descriptor.gatt.characteristic_user_description
    clawDesc->setValue("Claw Arm");

    pHorizontalArmAxisChar->setCallbacks(new ServoAxisCallbacks(&horizontal, vHorizontalRange[0], vHorizontalRange[1], "Horizontal"));
    pHorizontalArmAxisChar->addDescriptor(horizontalDesc);
    pHorizontalArmAxisChar->addDescriptor(horizontalRange);

    pVerticalArmAxisChar->setCallbacks(new ServoAxisCallbacks(&vertical, vVerticalRange[0], vVerticalRange[1], "Vertical"));
    pVerticalArmAxisChar->addDescriptor(verticalDesc);
    pVerticalArmAxisChar->addDescriptor(verticalRange);

    pDistanceArmAxisChar->setCallbacks(new ServoAxisCallbacks(&distance, vDistanceRange[0], vDistanceRange[1], "Distance"));
    pDistanceArmAxisChar->addDescriptor(distanceDesc);
    pDistanceArmAxisChar->addDescriptor(distanceRange);

    pClawArmAxisChar->setCallbacks(new ServoAxisCallbacks(&claw, vClawRange[0], vClawRange[1], "Claw"));
    pClawArmAxisChar->addDescriptor(clawDesc);
    pClawArmAxisChar->addDescriptor(clawRange);

    pArmService->start();
}

void setup_advertising() {
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(ROBOT_SERVICE_UUID);
    pAdvertising->addServiceUUID(ARM_SERVICE_UUID);
    pAdvertising->addServiceUUID(BATTERY_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();
}

void setup() {
    digitalWrite(LED, HIGH);
    Serial.begin(115200); // debug purposes

    // 32 horizontal 500 2500
    // 26 vertical 1000 2500
    // 33 distance 1250 2500
    // 25 claw 1500 2500 
    horizontal.attach(32, 500, 2500);
    vertical.attach(26, 1500, 3000);
    distance.attach(33, 1350, 2500);
    claw.attach(25, 2000, 3000);

    BLEDevice::init("BLE Robot Tank"); // This also setup GAP Service...
    pServer = BLEDevice::createServer();

    setup_battery_service();
    setup_robot_service();
    setup_arm_service();
    setup_advertising();

    Serial.println("Characteristic defined! Now you can read it in your phone!");
    digitalWrite(LED, LOW);
}

void read_battery_level() {
    if (!client_connected)
        return;

    uint8_t newCapacity = battery.readCapacity();
    if (battery_capacity != newCapacity) {
        battery_capacity = newCapacity;
        pBatteryLevelChar->setValue((int &) battery_capacity);
        pBatteryLevelChar->notify();
    }
    Serial.print("Battery capacity: ");
    Serial.println(battery_capacity);
}

void loop() {
    if (client_connected) {
        digitalWrite(LED, HIGH);
    } else {
        digitalWrite(LED, LOW);
    }
    read_battery_level();
    delay(1000);
}
