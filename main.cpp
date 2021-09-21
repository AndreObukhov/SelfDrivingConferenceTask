#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <map>
#include <string>
#include <ctime>

#define N_LEFT_STEPS 10

class mapElem;
class Order;
/*
int symbolCount = 0;
void writeSymbol() {
    symbolCount ++;
    if (symbolCount == 60) {
        std::cout << std::endl;
        symbolCount = 0;
    }
}
*/

typedef struct point {
    point() {
        x = 0;
        y = 0;
    }

    point(const short & x_n, const short & y_n) {
        x = x_n;
        y = y_n;
    }

    short x;
    short y;
} point;

void printPoint(const point& p) {
    std::cout << p.x << " " << p.y << std::endl;
}

bool operator == (const point& p1, const point& p2) {
    if(p1.x == p2.x && p1.y == p2.y) {
        return true;
    } else {
        return false;
    }
}

point operator - (const point& p1, const point& p2) {
    point diff;
    diff.x = p1.x - p2.x;
    diff.y = p1.y - p2.y;
    return diff;
}

int distanceBetweenPoints(const point& p1, const point& p2) {
    return (abs(p1.x - p2.x) + abs(p1.y - p2.y));
}

class mapElem {
public:
    mapElem() {
        free = true;
    }

    mapElem(const char& c) {
        if(c == '.')
            free = true;
        else
            free = false;
    }

    void assign(const char& c) {
        if(c == '#')
            free = false;
    }

    bool isFree() const {return free;}

    char getFree() const {
        if(free)
            return '.';
        else
            return '#';
    }

    void setPrev(const point& p) {
        prevPoint.x = p.x;
        prevPoint.y = p.y;
    }

    void setVisited() {
        visited = true;
    }

    bool isVisited() { return visited; }

    point getPrev() {
        point a = prevPoint;
        prevPoint.x = 0;        // ?
        prevPoint.y = 0;        // ?
        return a;
    }

    void addOrder(const int& o) {
        orderIDs.push_back(o);
    }

    int getnOrders() const {
        return orderIDs.size();
    }

    int takeOrder() {
        if (orderIDs.empty()) {
            return -1;
        }

        int o = orderIDs.front();
        orderIDs.pop_front();
        return o;
    }

private:
    bool free;
    // int nOrders;
    point prevPoint;
    bool visited = false;
    std::deque<int> orderIDs;
};


// passing copy of map to build routes on it
std::deque<point> findRoute(std::vector<std::vector<mapElem>> cityMap,
                            const point& start, const point& end) {
    std::deque<point> route;
    if(start == end) {
        return route;
    }

    // all points in one step from current possible states (bfs)
    std::deque<point> nextPoints;
    nextPoints.push_back(start);

    while(!nextPoints.empty()) {
        point currentPoint = nextPoints.front();
        cityMap[currentPoint.x][currentPoint.y].setVisited();

        // The point is end => building the route
        if(currentPoint == end) {
            // std::cout << "Building route" << std::endl;
            // build route
            point pt = end;
            while(!(pt == start)) {
                route.push_front(pt);
                pt = cityMap[pt.x][pt.y].getPrev();
            }
            // route.push_front(start);
            break;
        }

        // The point is not the end => visiting four neighbours
        // moving up
        if (currentPoint.y - 1 >= 0 &&
            cityMap[currentPoint.x][currentPoint.y - 1].isFree() &&
            !cityMap[currentPoint.x][currentPoint.y - 1].isVisited())
        {
            cityMap[currentPoint.x][currentPoint.y - 1].setPrev(currentPoint);
            cityMap[currentPoint.x][currentPoint.y - 1].setVisited();
            nextPoints.emplace_back(currentPoint.x, currentPoint.y - 1);
        }
        // moving down
        if (currentPoint.y + 1 < cityMap.size() &&
            cityMap[currentPoint.x][currentPoint.y + 1].isFree() &&
            !cityMap[currentPoint.x][currentPoint.y + 1].isVisited())
        {
            cityMap[currentPoint.x][currentPoint.y + 1].setPrev(currentPoint);
            cityMap[currentPoint.x][currentPoint.y + 1].setVisited();
            nextPoints.emplace_back(currentPoint.x, currentPoint.y + 1);
        }
        // moving left
        if (currentPoint.x - 1 >= 0 &&
            cityMap[currentPoint.x - 1][currentPoint.y].isFree() &&
            !cityMap[currentPoint.x - 1][currentPoint.y].isVisited())
        {
            cityMap[currentPoint.x - 1][currentPoint.y].setPrev(currentPoint);
            cityMap[currentPoint.x - 1][currentPoint.y].setVisited();
            nextPoints.emplace_back(currentPoint.x - 1, currentPoint.y);
        }
        // moving right
        if (currentPoint.x + 1 < cityMap.size() &&
            cityMap[currentPoint.x + 1][currentPoint.y].isFree() &&
            !cityMap[currentPoint.x + 1][currentPoint.y].isVisited())
        {
            cityMap[currentPoint.x + 1][currentPoint.y].setPrev(currentPoint);
            cityMap[currentPoint.x + 1][currentPoint.y].setVisited();
            nextPoints.emplace_back(currentPoint.x + 1, currentPoint.y);
        }
        nextPoints.pop_front();
        // for (point p : nextPoints) {
        //    printPoint(p);
        // }
        // std::cout << "-----" << std::endl;
    }
    return route;
}

void printCityMap(const std::vector<std::vector<mapElem>>& cityMap) {
    for (const auto& vec : cityMap) {
        for (const auto& c : vec) {
            std::cout << c.getFree() << " ";
        }
        std::cout << std::endl;
    }
}


class Order {
public:
    Order() {
        orderTime = 0;
        start.x = 0;
        start.y = 0;
        finish.x = 0;
        finish.y = 0;
        // orderID = 0;
        // isAssigned = false;
        isBeingDelivered = false;
    }
    Order(int xStart, int yStart, int xFinish, int yFinish, int t, int ID) {
        orderTime = t;
        start.x = xStart;
        start.y = yStart;
        finish.x = xFinish;
        finish.y = yFinish;
        // orderID = ID;
        // isAssigned = false;
        isBeingDelivered = false;
    }

    int calculateReward(const int& maxReward, const int& currentTime) const {
        int r = (maxReward - (currentTime - orderTime) - route.size());
        if (r > 0) {
            return r;
        } else {
            return 0;
        }
    }

    void findOrderRoute(const std::vector<std::vector<mapElem>>& cityMap) {
        route = findRoute(cityMap, start, finish);
    }

    void assignOrder(const short& ID) {
        // isAssigned = true;
        robotID = ID;
    }

    void takeOrder() { isBeingDelivered = true;}

    int orderTime;
    point start;
    point finish;
    // int orderID;
    // bool isAssigned;
    bool isBeingDelivered;
    std::deque<point> route;
    // int routeTime;
    short robotID = 0;
};


class Robot {
public:
    Robot(const short& ID, const point& pos) {
        robotID = ID;
        position = pos;
        orderID = 0;
        free = true;
        withOrder = false;
    }

    Robot() {
        robotID = 0;
        position = point(0, 0);
        orderID = 0;
        free = true;
        withOrder = false;
    }

    bool isFree() const {
        return free;
    }
    bool isWithOrder() const {
        return withOrder;
    }
    short getID() const {
        return robotID;
    }
    point getPosition() const {
        return position;
    }
    int getStepsLeft() const {
        return route.size();
    }
    point getFinishPoint() const {
        return route.back();
    }
    void setRoute(const std::deque<point>& r) {
        for (const point& pt: r) {
            route.push_back(pt);
        }
        free = false;
    }
    void setRouteForOrder(const std::deque<point>& r) {
        for (const point& pt: r) {
            routeForOrder.push_back(pt);
        }
    }

    void dropOrder() {
        // std::cout << "P";
        orderID = 0;
        free = true;
        withOrder = false;
    }
    void takeOrder(const int& newOrderID) {
        // std::cout << "T";
        orderID = newOrderID;
        free = false;
        withOrder = true;

        if (!routeForOrder.empty()) {
            setRoute(routeForOrder);
            routeForOrder.clear();
        }
    }

    char makeMove(std::vector<std::vector<mapElem>>& cityMap,
                  std::map<int, Order>& currentOrders) {
        if (route.empty()) {
            if (withOrder) {
                // arrived at destination with order => dropping order
                dropOrder();
                // writeSymbol();
                return 'P';
            }
            if (!withOrder) {
                if(cityMap[position.x][position.y].getnOrders() > 0) {
                    // if there is an order in this point => taking order from the point
                    orderID = cityMap[position.x][position.y].takeOrder();
                    route = currentOrders[orderID].route;
                    // erasing order from pending orders map:
                    currentOrders.erase(orderID);
                    takeOrder(orderID);
                    // writeSymbol();
                    return 'T';
                } else {
                    // no route => staying in current place
                    // std::cout << "S";
                    // writeSymbol();
                    return 'S';
                }
            }
        } else {
            point next = route.front();
            route.pop_front();
            point diff = next - position;
            position = next;
            // check valid move?
            if (diff == point(0, 1)) {
                //std::cout << "R";
                return 'R';
            }
            if (diff == point(0, -1)) {
                // std::cout << "L";
                return 'L';
            }
            if (diff == point(1, 0)) {
                // std::cout << "D";
                return 'D';
            }
            if (diff == point(-1, 0)) {
                // std::cout << "U";
                return 'U';
            }
            // writeSymbol();
        }
        return 'S';
    }

private:
    short robotID;
    point position;
    bool free;
    bool withOrder;
    int orderID;
    std::deque<point> route;
    std::deque<point> routeForOrder;
};


class City {
public:
    int citySize;
    std::vector<std::vector<mapElem>> cityMap;
    int maxReward;
    short nRobots;
    std::vector<Robot> cityRobots;
    // std::deque<Order> cityOrders;
    std::map<int, Order> currentOrders;
    std::map<short, std::string> robotMoves;

    City() {
        citySize = 2;
        nRobots = 1;
        maxReward = 0;
    }

    void readCityMap(const int& N) {
        citySize = N;
        cityMap.resize(citySize, std::vector<mapElem>(citySize));
        for (int i = 0; i < citySize; i ++) {
            for (int j = 0; j < citySize; j ++) {
                char c;
                std::cin >> c;
                // std::cout << c << std::endl;
                cityMap[i][j].assign(c);
            }
        }
    }

    void generateRobots(const short& nR) {
        nRobots = nR;
        // cityRobots.resize(nR);
        int n_rows = int(sqrt(nR)) + 1;
        // std::cout << "N rows: " << n_rows << std::endl;

        for (short robotID = 0; robotID < nRobots; robotID ++) {
            // some kind of equal distribution on the map
            int x = (citySize / (n_rows + 1)) * (1 + robotID / n_rows);
            int y = (citySize / (n_rows + 1)) * (1 + robotID % n_rows);

            point robotPos;

            // std::cout << "hi" << std::endl;

            if (cityMap[x][y].isFree()) {
                // if valid point => assigning this place to robot
                robotPos.x = x;
                robotPos.y = y;
            }
            else {
                // if invalid point => searching random free space for robot
                while(!cityMap[x][y].isFree()) {
                    x = rand() % citySize;
                    y = rand() % citySize;
                }
                robotPos.x = x;
                robotPos.y = y;
            }

            // Robot newRobot(robotID, robotPos);
            // std::cout << newRobot.getID() << std::endl;
            cityRobots.emplace_back(Robot(robotID, robotPos));
        }
    }

    void readOrder(int& orderID, const int& time) {
        int xStart, yStart;
        int xEnd, yEnd;
        std::cin >> xStart >> yStart >> xEnd >> yEnd;
        Order o(xStart - 1, yStart - 1, xEnd - 1, yEnd - 1, time, orderID);
        o.findOrderRoute(cityMap);

        // adding formed order into pending orders set
        currentOrders[orderID] = o;

        // adding orderID in the corresponding map cell
        cityMap[xStart - 1][yStart - 1].addOrder(orderID);
        orderID ++;
    }

    short findClosestRobot(const point& p) {
        short closestID = -1;
        int minDistance = 10000;
        for (const Robot& r : cityRobots) {
            if(r.isFree()) {                        // || (r.isWithOrder() && r.getStepsLeft() < N_LEFT_STEPS)
                point pos = r.getPosition();
                int dist = distanceBetweenPoints(pos, p);
                // if (r.isWithOrder()) {
                //    dist = distanceBetweenPoints(p, r.getFinishPoint());
                // }
                if(dist < minDistance) {
                    closestID = r.getID();
                    minDistance = dist;
                }
            }
        }
        return closestID;
    }

    void makeIteration(const int& time) {
        if (!currentOrders.empty()) {
            // new unhandled orders => need to assign them
            int mostProfitableOrder = -1;
            int maxProfit = 0;
            for (auto& [k, v]: currentOrders) {
                short freeRobotID = findClosestRobot(v.start);
                if (freeRobotID != - 1) {
                    int rew = v.calculateReward(maxReward, time);
                    if (rew > maxProfit) {
                        maxProfit = rew;
                        mostProfitableOrder = k;
                        v.assignOrder(freeRobotID);
                    }
                }
            }
            if (mostProfitableOrder >= 0) {
                short robotID = currentOrders[mostProfitableOrder].robotID;
                point orderPos = currentOrders[mostProfitableOrder].start;
                point robotPos = cityRobots[robotID].getPosition();
                std::deque<point> route = findRoute(cityMap, robotPos, orderPos);
                cityRobots[robotID].setRoute(route);
                cityRobots[robotID].setRouteForOrder(currentOrders[mostProfitableOrder].route);
                currentOrders.erase(mostProfitableOrder);
            }
        }

        for (Robot& r: cityRobots) {
            char move = r.makeMove(cityMap, currentOrders);
            robotMoves[r.getID()].push_back(move);
        }
    }

    void dumpMoves() {
        for (const auto& [rID, str]: robotMoves) {
            std::cout << str << std::endl;
        }
        for (short i = 0; i < nRobots; i ++) {
            robotMoves[i].clear();
        }
    }

    void printOrders() {
        for (const auto& [k, v]: currentOrders) {
            std::cout << k << ":";
            printPoint(v.start);
            std::cout << std::endl;
        }
    }
};


int main() {
    std::time_t start_time = time(nullptr);
    // std::cout << start_time << std::endl;

    City city;

    int N;                  // city map is NxN
    int maxTips;            // max profit for one order delivered
    int robotCost;          // cost of building 1 robot
    std::cin >> N >> maxTips >> robotCost;

    city.readCityMap(N);
    city.maxReward = maxTips;

    int T;                  // number of iterations
    int D;                  // number of orders
    std::cin >> T >> D;

    int nRobots = D * (maxTips - 2 * N) / (robotCost * 4);
    if (nRobots < 1) {
        nRobots = 1;
    }
    if(nRobots > 100) {
        nRobots = 100;
    }
    if (robotCost > 50000) {
        nRobots = 1;
    }

    city.generateRobots(nRobots);
    std::cout << nRobots << std::endl;
    for (const Robot& r: city.cityRobots) {
        point p = r.getPosition();
        p.x ++;
        p.y ++;
        printPoint(p);
    }
    /*
    for (const auto& r: city.cityRobots) {
        point pos = r.getPosition();
        std::cout << pos.x << ":" << pos.y << std::endl;
    }
    */
    // printCityMap(city.cityMap);

    int t = 0;              // current time
    int orderID = 0;        // unique ID for each given order
    int nOrdersOnStep = 0;  // how many new orders are given on each step

    for( ; t < T; t ++) {
        if (time(nullptr) - start_time > 17) {
            if (orderID < D) {
                std::cin >> nOrdersOnStep;
                if (nOrdersOnStep > 0) {
                    for(int i = 0; i < nOrdersOnStep; i ++) {
                        orderID++;
                        int xStart, yStart;
                        int xEnd, yEnd;
                        std::cin >> xStart >> yStart >> xEnd >> yEnd;
                    }
                }
            }

            for (int k = 0; k < nRobots; k ++) {
                std::cout << "SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS" << std::endl;
            }
            continue;
        }

        if (orderID < D) {
            std::cin >> nOrdersOnStep;
            if (nOrdersOnStep > 0) {
                for(int i = 0; i < nOrdersOnStep; i ++) {
                    city.readOrder(orderID, t);
                }
            }
        }

        for (int j = 0; j < 60; j ++)
            city.makeIteration(t);
        city.dumpMoves();
    }

    /*
    point p1 = {0, 2};
    point p2 = {0, 0};
    std::deque<point> r = findRoute(cityMap, p1, p2);
    for(point p : r) {
        printPoint(p);
    }
    // std::cout << (p1 == p2);
    */
    return 0;
}
