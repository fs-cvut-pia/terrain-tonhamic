#include "Path.h"
#include <cmath>
#include <queue>
#include <algorithm>
#include <map>
#include <set>
#include <fstream>
#include <iostream>

Path::Path(TerrainMap& m, std::string name_in, Point start_in, Point finish_in) : map(m), name(name_in), start(start_in), finish(finish_in) {};

void Path::printStats() const {
    bool land = false;
    bool water = false;
    double length = 0.0;
    double alt = 0.0;

    if (path.size() == 0) {
        std::cout << "Path empty." << std::endl;
        return;
    }

    int max_alt = map.alt(path[0]);

    for (int i=1; i<path.size(); ++i) {
        Point u = path[i];
        Point u_prev = path[i-1];
        if (i < path.size() - 1 && map.alt(u) > 0) land = true;
        if (map.alt(u) < 0) water = true;
        length += (u - u_prev).length();
        alt += std::abs(map.alt(u) - map.alt(u_prev));
        if (map.alt(u) > max_alt) max_alt = map.alt(u);
    }

    std::cout << "Path designated start = [" << start.x << ", " << start.y << "], finish = [" << finish.x << ", " << finish.y << "]" << std::endl;

    if (path[0] != start)
        std::cout << "First point on path [" << path[0].x << ", " << path[0].y << "] does not correspond to the designated starting point [" << start.x << ", " << start.y << "] !" << std::endl;

    if (path[path.size()-1] != finish)
        std::cout << "Last point on path [" << path[path.size()-1].x << ", " << path[path.size()-1].y << "] does not correspond to the designated finish point [" << finish.x << ", " << finish.y << "] !" << std::endl;

    if (land) std::cout << "Path type: land" << std::endl;
    if (water) std::cout << "Path type: water" << std::endl;
    std::cout << "Path length: " << length << " km" << std::endl;
    std::cout << "Total elevation gain + loss: " << alt << " m" << std::endl;
    std::cout << "Max. elevation: " << max_alt << " m" << std::endl;
}

std::string Path::getName() const {
    return name;
}

void Path::saveToFile() const {
    std::ofstream output(name + ".dat");

    if (!output) throw std::runtime_error("Cannot open file ../" + name + ".dat");

    for (Point u : path) {
        output << u.x << " " << u.y << std::endl;
    }
}

AirplanePath::AirplanePath(TerrainMap& m, std::string name_in, Point start_in, Point finish_in)
    : Path(m, name_in, start_in, finish_in) {}

bool AirplanePath::find() {
    // Check if start and finish are valid
    if (this->map.alt(this->start) <= 0 || this->map.alt(this->finish) <= 0) {
        return false;
    }

    //BFS algorithm, without considering the elevation
    std::queue<Point> q;
    std::map<Point, Point> parent;
    std::set<Point> visited;

    q.push(this->start);
    visited.insert(this->start);

    const int dx[] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int dy[] = {-1,  0,  1, -1, 1, -1, 0, 1};
    const double distances[] = {std::sqrt(2), 1, std::sqrt(2), 1, 1, std::sqrt(2), 1, std::sqrt(2)};

    while (!q.empty()) {
        Point current = q.front();
        q.pop();

        if (current == this->finish) {
            this->path.clear();
            Point backtrack = this->finish;
            while (backtrack != this->start) {
                this->path.push_back(backtrack);
                backtrack = parent[backtrack];
            }
            this->path.push_back(this->start);
            std::reverse(this->path.begin(), this->path.end());
            return true;
        }

        for (int i = 0; i < 8; i++) {
            Point next(current.x + dx[i], current.y + dy[i]);
            if (this->map.validCoords(next) && visited.find(next) == visited.end()) {
                q.push(next);
                visited.insert(next);
                parent[next] = current;
            }
        }
    }
    return false;
}

CarPath::CarPath(TerrainMap& m, std::string name_in, Point start_in, Point finish_in)
    : Path(m, name_in, start_in, finish_in) {}

bool CarPath::find() {
    
    // Check if start and finish are valid
    if (this->map.alt(this->start) <= 0 || this->map.alt(this->finish) <= 0) {
        return false;
    }
    //BFS algorithm
    std::queue<Point> q;
    std::map<Point, Point> parent;
    std::set<Point> visited;

    q.push(this->start);
    visited.insert(this->start);

    const int dx[] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int dy[] = {-1,  0,  1, -1, 1, -1, 0, 1};
    const double distances[] = {std::sqrt(2), 1, std::sqrt(2), 1, 1, std::sqrt(2), 1, std::sqrt(2)};

    while (!q.empty()) {
        Point current = q.front();
        q.pop();

        if (current == this->finish) {
            this->path.clear();
            Point backtrack = this->finish;
            while (backtrack != this->start) {
                this->path.push_back(backtrack);
                backtrack = parent[backtrack];
            }
            this->path.push_back(this->start);
            std::reverse(this->path.begin(), this->path.end());
            return true;
        }

        for (int i = 0; i < 8; i++) {
            Point next(current.x + dx[i], current.y + dy[i]);
            //we need to additionally check if the point is on the land and not in water
            if (this->map.validCoords(next) && visited.find(next) == visited.end() && this->map.alt(next) > 0) {
                double elevation_diff = std::abs(this->map.alt(next) - this->map.alt(current)); //elevation in meters
                double distance = distances[i] * 1000.0;  //distance in meters
                //if the elevation difference is less than 6%, the point is valid for the path
                if (elevation_diff / distance < 0.06) {
                    q.push(next);
                    visited.insert(next);
                    parent[next] = current;
                }
            }
        }
    }
    return false;
}

ShipPath::ShipPath(TerrainMap& m, std::string name_in, Point start_in, Point finish_in)
    : Path(m, name_in, start_in, finish_in) {}

bool ShipPath::find() {
    std::queue<Point> q;
    std::map<Point, Point> parent;
    std::set<Point> visited;

    // No need to check if start and finish points are in water anymore
    q.push(this->start);
    visited.insert(this->start);

    const int dx[] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int dy[] = {-1,  0,  1, -1, 1, -1, 0, 1};
    const double distances[] = {std::sqrt(2), 1, std::sqrt(2), 1, 1, std::sqrt(2), 1, std::sqrt(2)};

    while (!q.empty()) {
        Point current = q.front();
        q.pop();

        if (current == this->finish) {
            this->path.clear();
            Point backtrack = this->finish;
            while (backtrack != this->start) {
                this->path.push_back(backtrack);
                backtrack = parent[backtrack];
            }
            this->path.push_back(this->start);
            std::reverse(this->path.begin(), this->path.end());
            return true;
        }
        //we need to additionally check if the point is in water
        for (int i = 0; i < 8; i++) {
            Point next(current.x + dx[i], current.y + dy[i]);
            // Allow any valid point if it's start or finish, otherwise must be water
            if (this->map.validCoords(next) && visited.find(next) == visited.end() && 
                (next == this->finish || next == this->start || this->map.alt(next) < 0)) {
                q.push(next);
                visited.insert(next);
                parent[next] = current;
            }
        }
    }
    return false;
}

FerryPath::FerryPath(TerrainMap& m, std::string name_in, Point start_in, Point finish_in)
    : Path(m, name_in, start_in, finish_in) {}

bool FerryPath::find() {
    std::queue<Point> q;
    std::map<Point, Point> parent;
    std::set<Point> visited;

    q.push(this->start);
    visited.insert(this->start);

    const int dx[] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int dy[] = {-1,  0,  1, -1, 1, -1, 0, 1};
    const double base_distances[] = {std::sqrt(2), 1, std::sqrt(2), 1, 1, std::sqrt(2), 1, std::sqrt(2)};

    while (!q.empty()) {
        Point current = q.front();
        q.pop();

        if (current == this->finish) {
            this->path.clear();
            Point backtrack = this->finish;
            while (backtrack != this->start) {
                this->path.push_back(backtrack);
                backtrack = parent[backtrack];
            }
            this->path.push_back(this->start);
            std::reverse(this->path.begin(), this->path.end());
            return true;
        }

        for (int i = 0; i < 8; i++) {
            Point next(current.x + dx[i], current.y + dy[i]);
            if (this->map.validCoords(next) && visited.find(next) == visited.end()) {
                // Variable storing if the point is valid for the path
                bool valid_point = false;
                double distance = base_distances[i];
                
                // In water - accept all valid points and multiply distance by 4
                if (this->map.alt(next) < 0) {
                    valid_point = true;
                    distance *= 4.0;
                }
                // On land - check elevation constraint
                else {
                    double elevation_diff = std::abs(this->map.alt(next) - this->map.alt(current));
                    double distance_meters = distance * 1000.0;  // Convert km to m
                    if (elevation_diff / distance_meters < 0.06) {
                        valid_point = true;
                    }
                }

                if (valid_point) {
                    q.push(next);
                    visited.insert(next);
                    parent[next] = current;
                }
            }
        }
    }
    return false;
}

