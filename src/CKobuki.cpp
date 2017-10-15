#include "CKobuki.h"
#include "termios.h"
#include "errno.h"
#include <cstddef>
#include <iostream>
#include <stdexcept>


plot p;
static std::vector<float> vectorX;
static std::vector<float> vectorY;
static std::vector<float> vectorGyroTheta;

extern "C";
// obsluha tty pod unixom
int set_interface_attribs2 (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    //tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IGNBRK | INLCR | ICRNL | IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking2 (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        printf ("error %d setting term attributes", errno);
}






int CKobuki::connect(char * comportT)
{
    HCom= open(comportT,O_RDWR|O_NOCTTY|O_NONBLOCK);

    if ( HCom== -1 )
    {
        throw std::runtime_error("Kobuki nepripojeny\n");
        return HCom;

    }
    else
    {
        set_interface_attribs2 (HCom, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking2 (HCom, 0);                // set no blocking
        /*  struct termios settings;
          tcgetattr(HCom, &settings);

          cfsetospeed(&settings, B115200); // baud rate
          settings.c_cflag &= ~PARENB; // no parity
          settings.c_cflag &= ~CSTOPB; // 1 stop bit
          settings.c_cflag &= ~CSIZE;
          settings.c_cflag |= CS8 | CLOCAL; // 8 bits
          settings.c_lflag &= ~ICANON; // canonical mode
          settings.c_cc[VTIME]=1;
          settings.c_oflag &= ~OPOST; // raw output

          tcsetattr(HCom, TCSANOW, &settings); // apply the settings*/
        tcflush(HCom, TCOFLUSH);


        printf("Kobuki pripojeny\n");
        return HCom;
    }
}

unsigned char * CKobuki::readKobukiMessage()
{
    unsigned char buffer[1];
    ssize_t Pocet;
    buffer[0] = 0;
    unsigned char * null_buffer(0);
    //citame kym nezachytime zaciatok spravy
    do {
        Pocet=read(HCom,buffer,1);
    } while (buffer[0] != 0xAA);
    //mame zaciatok spravy (asi)
    if (Pocet == 1 && buffer[0] == 0xAA)
    {
        //citame dalsi byte
        do {

            Pocet=read(HCom,buffer,1);

        } while (Pocet != 1); // na linuxe -1 na windowse 0



        //a ak je to druhy byte hlavicky
        if (Pocet == 1 && buffer[0] == 0x55)
        {
            // precitame dlzku
            Pocet=read(HCom,buffer,1);

            // ReadFile(hCom, buffer, 1, &Pocet, NULL);
            if (Pocet == 1)
            {
                //mame dlzku.. nastavime vektor a precitame ho cely
                int readLenght = buffer[0];
                unsigned char *outputBuffer = (unsigned char*)calloc(readLenght+4,sizeof(char));
                outputBuffer[0] = buffer[0];
                int pct = 0;

                do
                {
                    Pocet = 0;
                    int readpoc = (readLenght+1  - pct);
                    Pocet=read(HCom,outputBuffer+1+pct,readpoc);

                    pct = pct + (Pocet == -1 ? 0 : Pocet);
                } while (pct != (readLenght+1 ));

                // tu si mozeme ceknut co chodi zo serial intefejsu Kobukiho
                // for(int i=0;i<outputBuffer[0]+2;i++)
                // {
                // 	printf("%x ",outputBuffer[i]);
                // }

                return outputBuffer;
            }
        }
    }

    return null_buffer;
}

int CKobuki::checkChecksum(unsigned char * data)
{//najprv hlavicku
    unsigned char chckSum = 0;
    for (int i = 0; i < data[0]+2; i++)
    {
        chckSum ^= data[i];
    }
    return chckSum;//0 ak je vsetko v poriadku,inak nejake cislo
}

void CKobuki::setLed(int led1, int led2)
{
    char message[8] = {static_cast<char>(0xaa),0x55,0x04,0x0c,0x02,0x00,static_cast<char>((led1+led2*4)%256),0x00};
    message[7] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6];
    uint32_t pocet;
    pocet=write(HCom,&message,8);
}

// tato funkcia nema moc sama o sebe vyznam, payload o tom, ze maju byt externe napajania aktivne musi byt aj tak v kazdej sprave...
void CKobuki::setPower(int value){
    if (value == 1) {
        char message[8] = {static_cast<char>(0xaa),0x55,0x04,0x0C,0x02,static_cast<char>(0xf0),static_cast<char>(0xAF)};
        uint32_t pocet;
        pocet = write(HCom,&message,8);
    }
}


void CKobuki::setTranslationSpeed(int mmpersec)
{
    char message[14] = { static_cast<char>(0xaa),0x55,0x0A,0x0c,0x02,static_cast<char>(0xf0),0x00,0x01,0x04,static_cast<char>(mmpersec%256),static_cast<char>(mmpersec>>8),0x00,0x00,  0x00 };
    message[13] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^ message[10] ^ message[11] ^ message[12];

    uint32_t pocet;
    pocet=write(HCom,&message,14);

}

void CKobuki::setRotationSpeed(double radpersec)
{
    int speedvalue = radpersec * 230.0f / 2.0f;
    char message[14] = { static_cast<char>(0xaa),0x55,0x0A,0x0c,0x02,static_cast<char>(0xf0),0x00,0x01,0x04,static_cast<char>(speedvalue % 256),static_cast<char>(speedvalue >>8),0x01,0x00,  0x00 };
    message[13] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^ message[10] ^ message[11] ^ message[12];

    uint32_t pocet;
    pocet=write(HCom,&message,14);
}

void CKobuki::setArcSpeed(int mmpersec, int radius)
{
    if (radius == 0) {
        setTranslationSpeed(mmpersec);
        return;
    }

    int speedvalue = mmpersec * ((radius + (radius>0? 230:-230) )/ 2 ) / radius;
    char message[14] = { static_cast<char>(0xaa),0x55,0x0A,0x0c,0x02,static_cast<char>(0xf0),0x00,0x01,0x04,static_cast<char>(speedvalue % 256),static_cast<char>(speedvalue >>8),static_cast<char>(radius % 256),static_cast<char>(radius >>8),  0x00 };
    message[13] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^ message[10] ^ message[11] ^ message[12];
    uint32_t pocet;
    pocet=write(HCom,&message,14);
}

void CKobuki::setSound(int noteinHz, int duration)
{
    int notevalue = floor((double)1.0 / ((double)noteinHz*0.00000275) + 0.5);
    char message[9] = { static_cast<char>(0xaa),0x55,0x05,0x03,0x03,static_cast<char>(notevalue%256),static_cast<char>(notevalue>>8),static_cast<char>(duration%256),0x00 };
    message[8] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7];

    uint32_t pocet;
    pocet=write(HCom,&message,28);
}




void CKobuki::startCommunication(char * portname, bool CommandsEnabled,  void *userDataL)
{
    connect(portname);
    enableCommands(CommandsEnabled);
    userData = userDataL;

    int pthread_result;
    pthread_result = pthread_create(&threadHandle,NULL,KobukiProcess,(void *)this);
}





int CKobuki::measure()
{
    while (stopVlakno==0)
    {
        unsigned char *message = readKobukiMessage();
        if (message == NULL)
        {
            printf("vratil null message\n");
            continue;
        }
        int ok=parseKobukiMessage(data,message);

        //maximalne moze trvat callback funkcia 20 ms, ak by trvala viac, nestihame citat
        if (ok == 0)
        {
            loop(userData, data);
        }
        free(message);
    }
    return 0;
}

int CKobuki::parseKobukiMessage(TKobukiData &output, unsigned char * data)
{
    int rtrnvalue = checkChecksum(data);
    //ak je zly checksum,tak kaslat na to
    if (rtrnvalue != 0)
        return -2;

    int checkedValue = 1;
    //kym neprejdeme celu dlzku
    while (checkedValue < data[0])
    {
        //basic data subload
        if (data[checkedValue] == 0x01)
        {
            checkedValue++;
            if (data[checkedValue ] != 0x0F)
                return -1;
            checkedValue++;
            output.timestamp = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.BumperCenter = data[checkedValue ] && 0x02;
            output.BumperLeft = data[checkedValue] && 0x04;
            output.BumperRight = data[checkedValue] && 0x01;
            checkedValue++;
            output.WheelDropLeft= data[checkedValue] && 0x02;
            output.WheelDropRight = data[checkedValue] && 0x01;
            checkedValue++;
            output.CliffCenter = data[checkedValue] && 0x02;
            output.CliffLeft = data[checkedValue] && 0x04;
            output.CliffRight = data[checkedValue] && 0x01;
            checkedValue++;
            output.EncoderLeft = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.EncoderRight = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.PWMleft = data[checkedValue] ;
            checkedValue++;
            output.PWMright = data[checkedValue] ;
            checkedValue++;
            output.ButtonPress = data[checkedValue];
            checkedValue++;
            output.Charger = data[checkedValue];
            checkedValue++;
            output.Battery = data[checkedValue];
            checkedValue++;
            output.overCurrent = data[checkedValue];
            checkedValue++;
        }
        else if (data[checkedValue] == 0x03)
        {
            checkedValue++;
            if (data[checkedValue] != 0x03)
                return -3;
            checkedValue++;
            output.IRSensorRight = data[checkedValue];
            checkedValue++;
            output.IRSensorCenter = data[checkedValue];
            checkedValue++;
            output.IRSensorLeft = data[checkedValue];
            checkedValue++;
        }
        else if (data[checkedValue] == 0x04)
        {
            checkedValue++;
            if (data[checkedValue] != 0x07)
                return -4;
            checkedValue++;
            output.GyroAngle = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.GyroAngleRate = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 5;//3 unsued
        }
        else if (data[checkedValue] == 0x05)
        {
            checkedValue++;
            if (data[checkedValue] != 0x06)
                return -5;
            checkedValue++;
            output.CliffSensorRight = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.CliffSensorCenter = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.CliffSensorLeft = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
        }
        else if (data[checkedValue] == 0x06)
        {
            checkedValue++;
            if (data[checkedValue] != 0x02)
                return -6;
            checkedValue++;
            output.wheelCurrentLeft =  data[checkedValue];
            checkedValue ++;
            output.wheelCurrentRight =data[checkedValue];
            checkedValue ++;

        }
        else if (data[checkedValue] == 0x0A)
        {
            checkedValue++;
            if (data[checkedValue] != 0x04)
                return -7;
            checkedValue++;
            output.extraInfo.HardwareVersionPatch = data[checkedValue];
            checkedValue++;
            output.extraInfo.HardwareVersionMinor = data[checkedValue];
            checkedValue++;
            output.extraInfo.HardwareVersionMajor = data[checkedValue];
            checkedValue += 2;

        }
        else if (data[checkedValue] == 0x0B)
        {
            checkedValue++;
            if (data[checkedValue] != 0x04)
                return -8;
            checkedValue++;
            output.extraInfo.FirmwareVersionPatch = data[checkedValue];
            checkedValue++;
            output.extraInfo.FirmwareVersionMinor = data[checkedValue];
            checkedValue++;
            output.extraInfo.FirmwareVersionMajor = data[checkedValue];
            checkedValue += 2;

        }
        else if (data[checkedValue] == 0x0D)
        {
            checkedValue++;
            if (data[checkedValue]%2 !=0)
                return -9;
            checkedValue++;
            output.frameId = data[checkedValue];
            checkedValue++;
            int howmanyFrames = data[checkedValue]/3;
            checkedValue++;
            output.gyroData.reserve(howmanyFrames);
            output.gyroData.clear();
            for (int hk = 0; hk < howmanyFrames; hk++)
            {
                TRawGyroData temp;
                temp.x = data[checkedValue + 1] * 256 + data[checkedValue];
                checkedValue += 2;
                temp.y = data[checkedValue + 1] * 256 + data[checkedValue];
                checkedValue += 2;
                temp.z = data[checkedValue + 1] * 256 + data[checkedValue];
                checkedValue += 2;
                output.gyroData.push_back(temp);
            }
        }
        else if (data[checkedValue] == 0x10)
        {
            checkedValue++;
            if (data[checkedValue] != 0x10)
                return -10;
            checkedValue++;
            output.digitalInput = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.analogInputCh0 = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.analogInputCh1 = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.analogInputCh2 = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 2;
            output.analogInputCh3 = data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 8;//2+6


        }
        else if (data[checkedValue] == 0x13)
        {
            checkedValue++;
            if (data[checkedValue] != 0x0C)
                return -11;
            checkedValue++;
            output.extraInfo.UDID0 = data[checkedValue + 3] * 256*256*256+ data[checkedValue + 2] * 256*256+ data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 4;
            output.extraInfo.UDID1 = data[checkedValue + 3] * 256 * 256 * 256 + data[checkedValue + 2] * 256 * 256 +data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 4;
            output.extraInfo.UDID2 = data[checkedValue + 3] * 256 * 256 * 256 + data[checkedValue + 2] * 256 * 256 +data[checkedValue + 1] * 256 + data[checkedValue];
            checkedValue += 4;
        }
        else
        {
            checkedValue++;
            checkedValue += data[checkedValue] + 1;
        }
    }
    return 0;
}







long double CKobuki::gyroToRad(signed short GyroAngle) {

    long double ret;
    if (GyroAngle < 0) {
        ret = GyroAngle + 36000;
    }
    else {
        ret = GyroAngle;
    }
    return (long double) ret*PI/18000.0;
}





long CKobuki::loop(void *user_data, TKobukiData &Kobuki_data) {
    if (iterationCount == 0) {
        prevLeftEncoder = Kobuki_data.EncoderLeft;
        prevRightEncoder = Kobuki_data.EncoderRight;
        prevTimestamp = Kobuki_data.timestamp;
        prevGyroTheta = gyroToRad(Kobuki_data.GyroAngle);
        iterationCount++;
    }

    int dLeft;
    if (abs(Kobuki_data.EncoderLeft - prevLeftEncoder) > 32000) {
        dLeft = Kobuki_data.EncoderLeft - prevLeftEncoder + (Kobuki_data.EncoderLeft > prevLeftEncoder ? -65536 : +65536);
    }
    else {
        dLeft = Kobuki_data.EncoderLeft - prevLeftEncoder;
    }

    int dRight;
    if (abs(Kobuki_data.EncoderRight - prevRightEncoder) > 32000) {
        dRight = Kobuki_data.EncoderRight - prevRightEncoder + (Kobuki_data.EncoderRight > prevRightEncoder ? -65536 : +65536);
    }
    else {
        dRight = Kobuki_data.EncoderRight - prevRightEncoder;
    }

    long double dGyroTheta = prevGyroTheta - gyroToRad(Kobuki_data.GyroAngle);

    if (dGyroTheta > PI) {
        dGyroTheta -= 2*PI;
    }
    if (dGyroTheta < -1*PI) {
        dGyroTheta += 2*PI;
    }

    gyroTheta += dGyroTheta;

    uint16_t dTimestamp = Kobuki_data.timestamp - prevTimestamp;

    long double mLeft = dLeft*tickToMeter;
    long double mRight = dRight*tickToMeter;

    if (mLeft == mRight) {
        x = x + mRight;
    } else {
        x = x + (b*(mRight+mLeft))/(2*(mRight-mLeft))*(sin((mRight-mLeft)/b + theta) - sin(theta));
        y = y + (b*(mRight+mLeft))/(2*(mRight-mLeft))*(cos((mRight-mLeft)/b + theta) - cos(theta));
        theta = (mRight-mLeft)/b + theta;
    }

    displacement = (mRight + mLeft)/2;
    integratedGyroTheta = integratedGyroTheta + dGyroTheta;
    gx = gx + displacement * cos(integratedGyroTheta + dGyroTheta / 2);

    gy = gy + displacement * sin(integratedGyroTheta + dGyroTheta / 2);






    totalLeft +=dLeft;
    totalRight +=dRight;

    // ak je suma novej a predchadzajucej vacsia ako 65536 tak to pretieklo?
    directionL = (prevLeftEncoder < Kobuki_data.EncoderLeft ? 1 : -1);
    directionR = (prevRightEncoder < Kobuki_data.EncoderRight ? 1 : -1);
    dTimestamp = (Kobuki_data.timestamp < prevTimestamp ? prevTimestamp - Kobuki_data.timestamp + 65536 : dTimestamp);


    prevLeftEncoder = Kobuki_data.EncoderLeft;
    prevRightEncoder = Kobuki_data.EncoderRight;
    prevTimestamp = Kobuki_data.timestamp;
    prevGyroTheta = gyroToRad(Kobuki_data.GyroAngle);





    static long counter = 0;

    vectorX.push_back(gx);
    vectorY.push_back(gy);
    vectorGyroTheta.push_back(gyroTheta);

    currentX = gx;
    currentY = gy;
    currentTheta = gyroTheta;

//    std::cout << "X: " << gx
//              << " Y: " << gy
//              << " Gyro heta: " << gyroTheta
//              << std::endl;

    if (counter % 100 == 0) {
        p.plot_data(vectorY, vectorX);
    }
    counter++;

    return 0;
}




// povie kobukimu ze ma ist niekolko metrov dopredu alebo dozadu, rozhoduje znamienko
// funkcia kompenzuje chodenie rovno pomocou regulatora, interne vyuziva setArcSpeed a
// ako spatnu vazbu pouziva data z enkoderov
void CKobuki::goStraight(long double distance){
    long double u_translation = 0; // riadena velicina, rychlost robota pri pohybe
    long double w_translation = distance; // pozadovana hodnota

    // parametre regulatora
    long double Kp_translation = 4000;
    long double e_translation = 0;
    int upper_thresh_translation = 600;
    int lower_thresh_translation = 60;
    int translation_start_gain = 30;

    long double u_rotation = 0; // riadena velicina
    long double w_rotation = 0;
    long double Kp_rotation = 40;
    long double e_rotation = 0;

    x = 0;
    y = 0;
    theta = 0;

    long i = 1;

    while (fabs(x - w_translation) > 0.01 && x<w_translation) {
        e_translation = w_translation - x;
        u_translation = Kp_translation * e_translation;

        e_rotation = w_rotation - theta;
        if (!e_rotation == 0) u_rotation = Kp_rotation / e_rotation;


        // limit translation speed
        if (u_translation > upper_thresh_translation)
            u_translation = upper_thresh_translation;
        if (u_translation < lower_thresh_translation)
            u_translation = lower_thresh_translation;

        // rewrite starting speed with line
        if (i < u_translation) {
            u_translation = i;
        }

        if (fabs(u_rotation) > 32767) {
            u_rotation = -32767;
        }

        if (u_rotation == 0) {
            u_rotation = -32767;
        }

        this->setArcSpeed(u_translation, u_rotation);

        usleep(25*1000);
        // increment starting speed
        i = i + translation_start_gain;
    }
    this->setTranslationSpeed(0);
}



/// metoda vykona rotaciu, rotuje sa pomocou regulatora, ako spatna vazba sluzi gyroskop,
/// kedze je radovo presnejsi ako enkodery
void CKobuki::doRotation(long double th) {
    long double u = 0; // riadena velicina, uhlova rychlost robota pri pohybe
    long double w = th; // pozadovana hodnota v radianoch
    long double Kp = PI/2;
    long double e = 0;
    int thresh = PI*0.7;

    theta = 0;
    x = 0;
    y = 0;
    gyroTheta = 0;

    long double i = 0;

    if (w > 0) {
        while (gyroTheta < w-0.015) {
            e = w - gyroTheta;
            u = Kp*e;

            if (u > thresh) u = thresh;
            if (u < 0.4) u = 0.4;

            if (i < u) {
                u = i;
            }

            std::cout << "Angle: " << gyroTheta << " required:" << w << std::endl;
            this->setRotationSpeed(-1*u);
            usleep(25*1000);
            i = i + 0.1;
        }
    }
    else  {
        while (gyroTheta > w + 0.015) {
            e = w - gyroTheta;
            u = Kp*e*-1;

            if (u > thresh) u = thresh;
            if (u < 0.4) u = 0.4;

            if (i < u) {
                u = i;
            }

            std::cout << "Angle: " << gyroTheta << " required:" << w << std::endl;
            this->setRotationSpeed(u);
            usleep(25*1000);
            i = i + 0.1;
        }
    }

    std::cout << "stop the fuck!" << std::endl;
    // usleep(25*1000);
    this->setRotationSpeed(0);
    usleep(25*1000);
}


// kombinuje navadzanie na suradnicu a rotaciu o uhol, realizuje presun na zvolenu suradnicu
// v suradnicovom systeme robota
void CKobuki::goToXy(long double xx, long double yy) {
    long double th;

    yy = yy*-1;

    th = atan2(yy,xx);
    doRotation(th);

    long double s = sqrt(pow(xx,2)+pow(yy,2));

    // resetnem suradnicovu sustavu robota
    x = 0;
    y = 0;
    iterationCount = 0;
    theta = 0;

    //std::cout << "mam prejst: " << s << "[m]" << std::endl;

    goStraight(s);

    usleep(25*1000);
    return;
}

