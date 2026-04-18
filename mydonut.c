#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

typedef struct point {
    float x; 
    float y; 
    float z;
} Point;

typedef struct discrete_point {
    int x;
    int y;
} DiscretePoint;

typedef struct directed_point {
    Point pos;
    Point normal;
} DirectedPoint;

typedef struct point_cloud {
    DirectedPoint* points;
    Point centre;
    int curr_size;
    int max_size;
} PointCloud;


void rotate_x(Point* p, float t) {
    float s = sin(t), c = cos(t);
    float y = c * p->y - s * p->z;
    float z = s * p->y + c * p->z;
    p->y = y;
    p->z = z;
}

void rotate_y(Point* p, float t) {
    float s = sin(t), c = cos(t);
    float x = c * p->x + s * p->z;
    float z = c * p->z - s * p->x;
    p->x = x;
    p->z = z;
}

void rotate_z(Point* p, float t) {
    float s = sin(t), c = cos(t);
    float x = c * p->x - s * p->y;
    float y = s * p->x + c * p->y;
    p->x = x;
    p->y = y;
}

// rotation around the centre of the point cloud
void rotate_pos(Point* p, Point* centre, float t_x, float t_y, float t_z) {
    p->x -= centre->x;
    p->y -= centre->y;
    p->z -= centre->z;
    rotate_x(p, t_x);
    rotate_y(p, t_y);
    rotate_z(p, t_z);
    p->x += centre->x;
    p->y += centre->y;
    p->z += centre->z;
}

// rotation around the origin as normals are just direction vectors
void rotate_normal(Point* n, float t_x, float t_y, float t_z) {
    rotate_x(n, t_x);
    rotate_y(n, t_y);
    rotate_z(n, t_z);
}

void rotate_point_cloud(PointCloud* cloud, float t_x, float t_y, float t_z) {
    for (int i=0; i<cloud->curr_size; i++) {
        DirectedPoint* p = &cloud->points[i];
        rotate_pos(&p->pos, &cloud->centre, t_x, t_y, t_z);
        rotate_normal(&p->normal, t_x, t_y, t_z);
    }
}

// theta = small circle, phi = big circle
PointCloud donut_point_cloud(float R1, float R2, int theta_steps, int phi_steps, float offset_z) {
    int total_points = theta_steps * phi_steps;
    PointCloud cloud;
    cloud.points = (DirectedPoint*) calloc(total_points, sizeof(DirectedPoint));
    cloud.curr_size = 0;
    cloud.max_size = total_points;
    cloud.centre = (Point) {0, 0, offset_z};

    for (int i = 0; i < theta_steps; i++) {
        float theta = i * (2.0 * M_PI / theta_steps);
        for (int j = 0; j < phi_steps; j++) {
            float phi = j * (2.0 * M_PI / phi_steps);

            float x = (R1 + R2 * cos(theta)) * cos(phi);
            float y = (R1 + R2 * cos(theta)) * sin(phi);
            float z = R2 * sin(theta) + offset_z;
            float nx = cos(theta) * cos(phi);
            float ny = cos(theta) * sin(phi);
            float nz = sin(theta);
            cloud.points[cloud.curr_size++] = (DirectedPoint) {{x, y, z}, {nx, ny, nz}};
        }
    }
    return cloud;
}

// similar triangles
Point project_point_to_xy_plane(Point* p, float plane_z) {
    float x = p->x * plane_z / p->z;
    float y = p->y * plane_z / p->z;
    return (Point) {x, y, plane_z};
}

DiscretePoint discretize_screen_point(Point* p, int screen_size_x, int screen_size_y) {
    int x = (int) p->x + screen_size_x / 2; // float -> int conversion truncates
    int y = (int) p->y + screen_size_y / 2; // centre of the screen is at the origin
    return (DiscretePoint) {x, y};
}

// calculates the closest point to the viewer (located at the origin) on the z axis for each discrete point in the screen
// y-axis up is negative for consistency with translating discrete points to array indices
DirectedPoint* z_buffer(PointCloud* cloud, DiscretePoint screen_top_left, DiscretePoint screen_bottom_right, float screen_offset_z) {
    int screen_size_x = screen_bottom_right.x - screen_top_left.x;
    int screen_size_y = screen_bottom_right.y - screen_top_left.y;
    DirectedPoint* z_buffer = calloc(screen_size_x * screen_size_y, sizeof(DirectedPoint));
    
    for (int i=0; i<cloud->curr_size; i++) {
        DirectedPoint p = cloud->points[i];
        Point screen_p = project_point_to_xy_plane(&p.pos, screen_offset_z);
        
        if (screen_p.x < screen_top_left.x || screen_p.x >= screen_bottom_right.x || 
            screen_p.y < screen_top_left.y || screen_p.y >= screen_bottom_right.y) {
            continue;
        }

        DiscretePoint discrete_p = discretize_screen_point(&screen_p, screen_size_x, screen_size_y);

        int i = discrete_p.y * screen_size_x + discrete_p.x; // storing 2D array as 1D array with calloc
        if (z_buffer[i].pos.z == 0 || p.pos.z < z_buffer[i].pos.z) { // check for 0 because calloc initialize all memory to 0
            z_buffer[i] = p;
        }
    }

    return z_buffer;
}

// maps the DirectedPoint to a character by using the dot product formula on the point's normal and light direction
// light_dir points towards the light source (so if a point's normal is also pointing toward the light source, cos_theta is close to 1)
char brightness(Point* n, Point* light_dir, char* brightness_map, int len_map) {
    double mag_n = sqrt(n->x * n->x + n->y * n->y + n->z * n->z);
    if (mag_n == 0) {
        return brightness_map[0];
    }
    double cos_theta = (n->x * light_dir->x + n->y * light_dir->y + n->z * light_dir->z) / mag_n;
    cos_theta = (cos_theta + 1.0) / 2.0; // translate to positive values and squash into the range [0,1]
    int index = (int) (cos_theta * len_map);
    return brightness_map[index];
}

// map the zbuffer into an array of characters
char* brightness_mapped_screen(DirectedPoint* z_buffer, int screen_points, Point* light_dir, char* brightness_map, int len_map) {
    char* chars = calloc(screen_points, sizeof(char));
    for (int i=0; i<screen_points; i++) {
        DirectedPoint p = z_buffer[i];
        if (p.pos.z == 0) { // still 0 from calloc initialization, so no points mapped to this index
            chars[i] = ' ';
        }
        else {
            chars[i] = brightness(&p.normal, light_dir, brightness_map, len_map);
        }
    }
    return chars;
}

void print_screen(char* brightness_mapped_screen, int screen_size_x, int screen_size_y) {
    printf("\033[H");
    for (int y=0; y<screen_size_y; y++) {
        for (int x=0; x<screen_size_x; x++) {
            char c = brightness_mapped_screen[y * screen_size_x + x];
            printf("%c%c", c, c);
        }
        printf("\n");
    }
    fflush(stdout);
}

int main() {
    DiscretePoint screen_top_left = {-20, -20};
    DiscretePoint screen_bottom_right = {20, 20};
    int screen_size_x = screen_bottom_right.x - screen_top_left.x;
    int screen_size_y = screen_bottom_right.y - screen_top_left.y;
    int screen_points = screen_size_x * screen_size_y;
    float screen_offset_z = 15;
    Point light_dir = {0, -1, 0}; // y-axis inverted so -1 points upwards
    char* brightness_map = ".,-~:;=!*#$@";
    int len_map = 12;
    float theta_x = 0.0, theta_y = 0.0, theta_z = 0.0;

    while (1) {
        PointCloud cloud = donut_point_cloud(2, 0.67, 40, 90, 5);
        rotate_point_cloud(&cloud, theta_x, theta_y, theta_z);
        DirectedPoint* z_buff = z_buffer(&cloud, screen_top_left, screen_bottom_right, screen_offset_z);
        char* output = brightness_mapped_screen(z_buff, screen_points, &light_dir, brightness_map, len_map);
        print_screen(output, screen_size_x, screen_size_y);
        theta_x += 0.03;
        theta_y += 0.05;
        theta_z += 0.02;
        free(z_buff);
        free(output);
        free(cloud.points);
        usleep(30000);
    }

    return 0;
}
