#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <string>
using namespace std;

void Zadacha1();
void Zadacha2();
void Zadacha3();
void Zadacha4();
void Zadacha5();
void Zadacha6();
void Zadacha7();
void Zadacha8();
void Zadacha9();
void Zadacha10();
void Zadacha11();

// -------------------- ЗАДАЧА 1 --------------------
class Sensor {
private:
    double signalStrength;
    bool isActive;

public:
    Sensor(double strength) {
        setSignalStrength(strength);
        activate();
    }
    void activate() {
        isActive = true;
    }
    void deactivate() {
        isActive = false;
    }
    void setSignalStrength(double s) {
        if (s < 0) s = 0;
        signalStrength = s;
    }
    double getSignalStrength() {
        return signalStrength;
    }
    void printStatus() {
        cout << fixed << setprecision(1);
        cout << "Датчик ";
        cout << (isActive ? "активен" : "выключен");
        cout << " | Сила сигнала: " << signalStrength << " дБ\n";
    }
};

void Zadacha1() {
    cout << "\n--- Задача 1: Sensor ---\n";
    Sensor s1(45.5);
    s1.printStatus();
    s1.setSignalStrength(-10);
    s1.printStatus();
    s1.deactivate();
    s1.printStatus();
}

// -------------------- ЗАДАЧА 2 --------------------
class Trajectory {
private:
    double startX, startY;
    double angle;
public:
    Trajectory() : startX(0), startY(0), angle(0) {}
    Trajectory(double x, double y, double ang) {
        startX = x;
        startY = y;
        angle = ang;
    }
    void printTrajectory() {
        cout << "Старт: (" << startX << ", " << startY << "), угол: " << angle << "°\n";
    }
};

void Zadacha2() {
    cout << "\n--- Задача 2: Trajectory ---\n";
    Trajectory t1;
    Trajectory t2(100, 200, 45);
    t1.printTrajectory();
    t2.printTrajectory();
}

// -------------------- ЗАДАЧА 3 --------------------
class Engine {
private:
    double thrust;
    double fuelFlow;
public:
    Engine(double T, double F) {
        thrust = T;
        fuelFlow = F;
    }
    double getSpecificImpulse() {
        const double g = 9.81;
        return (fuelFlow == 0) ? 0 : thrust / (fuelFlow * g);
    }
    void printInfo() {
        cout << fixed << setprecision(2);
        cout << "Тяга: " << thrust << " Н | Расход: " << fuelFlow
            << " кг/с | Удельный импульс: " << getSpecificImpulse() << " с\n";
    }
};

void Zadacha3() {
    cout << "\n--- Задача 3: Engine ---\n";
    Engine e1(5000, 2.5);
    e1.printInfo();
}

// -------------------- ЗАДАЧА 4 --------------------
class Gyroscope {
private:
    double angularVelocity;
    bool calibrationRequired;
public:
    Gyroscope(double v, bool need) {
        angularVelocity = v;
        calibrationRequired = need;
    }
    void calibrate() { calibrationRequired = false; }
    void printStatus() {
        cout << fixed << setprecision(1);
        cout << "Скорость: " << angularVelocity << " °/с | Калибровка "
            << (calibrationRequired ? "требуется" : "не требуется") << "\n";
    }
};

void Zadacha4() {
    cout << "\n--- Задача 4: Gyroscope ---\n";
    Gyroscope g(120.5, true);
    g.printStatus();
    g.calibrate();
    g.printStatus();
}

// -------------------- ЗАДАЧА 5 --------------------
class Autopilot {
private:
    double courseAngle;
    double altitude;
public:
    Autopilot(double course, double alt) {
        courseAngle = course;
        altitude = alt;
    }
    void changeCourse(double d) { courseAngle += d; }
    void changeAltitude(double d) { altitude += d; }
    void printStatus() {
        cout << "Курс: " << courseAngle << "°, высота: " << altitude << " м\n";
    }
};

void Zadacha5() {
    cout << "\n--- Задача 5: Autopilot ---\n";
    Autopilot a(90, 1000);
    a.changeCourse(10);
    a.changeAltitude(500);
    a.printStatus();
}

// -------------------- ЗАДАЧА 6 --------------------
class RocketStage {
private:
    double thrust;
    double burnTime;
    double mass;
public:
    RocketStage(double T, double t, double m) {
        thrust = T;
        burnTime = t;
        mass = m;
    }
    double getDeltaV() {
        return (mass == 0) ? 0 : thrust * burnTime / mass;
    }
    void printInfo() {
        cout << fixed << setprecision(2);
        cout << "Ступень: тяга=" << thrust << ", время=" << burnTime
            << " с, масса=" << mass << " кг, ΔV=" << getDeltaV() << " м/с\n";
    }
};

void Zadacha6() {
    cout << "\n--- Задача 6: RocketStage ---\n";
    RocketStage s1(5000, 10, 1000);
    RocketStage s2(7000, 8, 900);
    RocketStage s3(9000, 6, 800);
    s1.printInfo();
    s2.printInfo();
    s3.printInfo();
    double total = s1.getDeltaV() + s2.getDeltaV() + s3.getDeltaV();
    cout << "Итоговая скорость: " << total << " м/с\n";
}

// -------------------- ЗАДАЧА 7 --------------------
class FlightComputer {
private:
    double altitude, velocity, fuel, thrust;
public:
    FlightComputer(double alt, double vel, double f, double thr) {
        altitude = alt; velocity = vel; fuel = f; thrust = thr;
    }
    void simulateStep(double dt) {
        if (fuel <= 0) return;
        velocity += (thrust / 1000 - 9.8) * dt;
        altitude += velocity * dt;
        fuel -= dt * 2;
        if (fuel < 0) fuel = 0;
    }
    void printStatus(double time) {
        cout << fixed << setprecision(1);
        cout << "t=" << time << "с: высота=" << altitude
            << " м, скорость=" << velocity
            << " м/с, топливо=" << fuel << " кг\n";
    }
};

void Zadacha7() {
    cout << "\n--- Задача 7: FlightComputer ---\n";
    FlightComputer fc(0, 0, 100, 12000);
    double time = 0, dt = 0.5;
    for (int i = 0; i < 5; i++) {
        time += dt;
        fc.simulateStep(dt);
        fc.printStatus(time);
    }
}

// -------------------- ЗАДАЧА 8 --------------------
class NavigationSystem {
private:
    double x, y;
    double vX, vY;
    bool gpsAvailable;
public:
    NavigationSystem(double sx, double sy, double vx, double vy, bool gps) {
        x = sx; y = sy; vX = vx; vY = vy; gpsAvailable = gps;
    }
    void integratePosition(double dt) {
        x += vX * dt;
        y += vY * dt;
    }
    void correctGPS(double realX, double realY) {
        if (gpsAvailable) {
            x = (x + realX) / 2;
            y = (y + realY) / 2;
        }
    }
    void printPosition() {
        cout << fixed << setprecision(1);
        cout << "Координаты: (" << x << ", " << y << ")\n";
    }
};

void Zadacha8() {
    cout << "\n--- Задача 8: NavigationSystem ---\n";
    NavigationSystem nav(0, 0, 100, 50, true);
    nav.integratePosition(1);
    nav.correctGPS(10, 60);
    nav.printPosition();
}

// -------------------- ЗАДАЧА 9 --------------------
class AutonomousControl {
private:
    double altitude, thrust, targetAltitude;
public:
    AutonomousControl(double alt, double thr, double target) {
        altitude = alt; thrust = thr; targetAltitude = target;
    }
    void updateControl() {
        if (altitude < targetAltitude) thrust += 100;
        else thrust -= 100;
        if (thrust < 0) thrust = 0;
    }
    void simulateStep(double dt) {
        altitude += (thrust / 500) * dt;
    }
    void printStatus() {
        cout << fixed << setprecision(0);
        cout << "Высота: " << altitude << " м, тяга: " << thrust << "\n";
    }
};

void Zadacha9() {
    cout << "\n--- Задача 9: AutonomousControl ---\n";
    AutonomousControl ac(0, 2000, 5000);
    for (int i = 0; i < 5; i++) {
        ac.updateControl();
        ac.simulateStep(1);
        ac.printStatus();
    }
}

// -------------------- ЗАДАЧА 10 --------------------
class DroneFlight {
private:
    vector<double> x, y;
    double totalDistance;
public:
    DroneFlight() {
        totalDistance = 0;
        x.push_back(0);
        y.push_back(0);
    }
    void addPoint(double nx, double ny) {
        double dx = nx - x.back();
        double dy = ny - y.back();
        totalDistance += sqrt(dx * dx + dy * dy);
        x.push_back(nx);
        y.push_back(ny);
    }
    void printPath() {
        cout << "Точки маршрута:\n";
        for (size_t i = 0; i < x.size(); i++)
            cout << "(" << x[i] << ", " << y[i] << ")\n";
    }
    double getTotalDistance() { return totalDistance; }
};

void Zadacha10() {
    cout << "\n--- Задача 10: DroneFlight ---\n";
    DroneFlight d;
    d.addPoint(0, 0);
    d.addPoint(3, 4);
    d.addPoint(6, 8);
    d.printPath();
    cout << "Пройдено: " << d.getTotalDistance() << " м\n";
}

// -------------------- ЗАДАЧА 11 --------------------
class Engine2 {
private:
    double thrust, fuelFlow, fuel;
public:
    Engine2(double thr, double flow, double f) {
        thrust = thr; fuelFlow = flow; fuel = f;
    }
    double getThrust() { return thrust; }
    bool hasFuel() { return fuel > 0; }
    void burn(double dt) {
        double fuelBurned = fuelFlow * dt;
        if (fuelBurned > fuel) fuelBurned = fuel;
        fuel -= fuelBurned;
    }
    double getFuel() { return fuel; }
};

class Navigation2 {
private:
    double altitude, velocity, acceleration, mass;
public:
    Navigation2(double alt, double vel, double m) {
        altitude = alt; velocity = vel; mass = m; acceleration = 0;
    }
    void update(double thrust, double dt) {
        acceleration = thrust / mass - 9.81;
        velocity += acceleration * dt;
        altitude += velocity * dt;
        if (altitude < 0) altitude = 0;
    }
    double getAltitude() { return altitude; }
    void printStatus(double t) {
        cout << fixed << setprecision(2);
        cout << "t=" << t << " c | Высота=" << altitude
            << " м | Скорость=" << velocity
            << " м/с | Ускорение=" << acceleration << " м/с²\n";
    }
};

class AutonomousFlightSystem {
private:
    Engine2 engine;
    Navigation2 nav;
    double time;
public:
    AutonomousFlightSystem(Engine2 e, Navigation2 n)
        : engine(e), nav(n), time(0) {
    }
    void simulate(double dt, double T) {
        nav.printStatus(time);
        while (time < T && engine.hasFuel()) {
            time += dt;
            engine.burn(dt);
            nav.update(engine.getThrust(), dt);
            nav.printStatus(time);
        }
        printSummary();
    }
    void printSummary() {
        cout << "--- Полёт завершён ---\n";
        cout << "Оставшееся топливо: " << engine.getFuel() << " кг\n";
        cout << "Конечная высота: " << nav.getAltitude() << " м\n";
    }
};

void Zadacha11() {
    cout << "\n--- Задача 11: AutonomousFlightSystem ---\n";
    Engine2 e(15000, 5, 50);
    Navigation2 n(0, 0, 1000);
    AutonomousFlightSystem s(e, n);
    s.simulate(1.0, 20.0);
}

// ---------------------------------------------------
int main() {
    setlocale(LC_ALL, "Russian");
    int choice;
    do {
        cout << "\n===== МЕНЮ ЗАДАЧ =====\n";
        cout << "1 - Sensor\n";
        cout << "2 - Trajectory\n";
        cout << "3 - Engine\n";
        cout << "4 - Gyroscope\n";
        cout << "5 - Autopilot\n";
        cout << "6 - RocketStage ΔV\n";
        cout << "7 - FlightComputer\n";
        cout << "8 - NavigationSystem\n";
        cout << "9 - AutonomousControl\n";
        cout << "10 - DroneFlight\n";
        cout << "11 - AutonomousFlightSystem\n";
        cout << "0 - Выход\n";
        cout << "Выберите задачу: ";
        cin >> choice;

        switch (choice) {
        case 1: Zadacha1(); break;
        case 2: Zadacha2(); break;
        case 3: Zadacha3(); break;
        case 4: Zadacha4(); break;
        case 5: Zadacha5(); break;
        case 6: Zadacha6(); break;
        case 7: Zadacha7(); break;
        case 8: Zadacha8(); break;
        case 9: Zadacha9(); break;
        case 10: Zadacha10(); break;
        case 11: Zadacha11(); break;
        case 0: cout << "Выход...\n"; break;
        default: cout << "Нет такой задачи!\n";
        }
    } while (choice != 0);

    return 0;
}
