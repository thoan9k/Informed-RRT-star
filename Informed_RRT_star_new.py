import pygame
import base_old
import time
import math
#from PIL import Image
import robot_follow_path as car
# import robot_follow_path_2_pid as car
start = (100, 200)
goal = (400, 500)
dimensions = (600, 600)
obsdim = 35 # kích thước đã phóng đại
adjust_obs = 0
obsnum =60
iteration_max = 1000

path_Cost1=[]
time1=[]
path_Cost2=[]
time2=[]
init_time = 0
def main(obs, graph):
    global start, goal, dimensions, obsdim, obsnum, iteration_max

    # tạo vật cản

    # vẽ vật cản
    graph.drawmap(obs)
    iteration = 0
    t1 = time.time()
    oldCost = float('inf')
    oldpath = None

    n_image = 0
    # # Lưu màn hình Pygame vào file BMP
    # pygame.image.save(map_.map, f"images/screenshot{n_image}.bmp")
    #
    # # Sử dụng Pillow để mở và chuyển file BMP sang PNG hoặc GIF
    # image = Image.open(f"images/screenshot{n_image}.bmp")
    # image.save(f"images/screenshot{n_image}.png")  # Lưu dưới dạng PNG
    # n_image += 1
    eslaped = 0
    start_time = time.time()
    init_time = start_time
    while True:
        # eslaped = time.time() - t1
        # t1 = time.time()
        # if eslaped >10:
            # raise
        quit_=False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit_=True
                break
        if quit_:
            break
        if iteration % 50 == 0:
            n = graph.number_of_nodes()
            x, y, parent = graph.bias()
            pygame.draw.circle(graph.surface, graph.red, (x[-1], y[-1]), graph.nodeRadian, graph.nodeThickness)
            pygame.draw.line(graph.surface, graph.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), graph.edgeThickness)
            # print(graph.number_of_nodes()-n)
            if graph.number_of_nodes() - n == 1:
                new_node = graph.number_of_nodes() - 1
                neighbors = graph.near_neighbors(new_node, kmax=10)
                graph.rewire(graph.surface, new_node, neighbors)

            # time.sleep(0.05)
        else:
            n = graph.number_of_nodes()
            x, y, parent = graph.expand()
            pygame.draw.circle(graph.surface, graph.grey, (x[-1], y[-1]), graph.nodeRadian, graph.nodeThickness)
            pygame.draw.line(graph.surface, graph.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), graph.edgeThickness)
            if graph.number_of_nodes() - n == 1:
                new_node = graph.number_of_nodes() - 1
                neighbors = graph.near_neighbors(new_node, kmax=10)
                graph.rewire(graph.surface, new_node, neighbors)
                # pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                #                  (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            # time.sleep(0.05)
        # time.sleep(0.05)
        pygame.display.update()
        iteration += 1
        # map_.drawpath(graph.getPathcoords())
        if iteration % 100 == 0:
            print(iteration)
        if graph.path_to_goal():
            new_Cost = graph.getfinalCost()
            if graph.type=='RRT_star':
                path_Cost1.append(new_Cost)
                time1.append(time.time()-init_time)
            else:
                path_Cost2.append(new_Cost)
                time2.append(time.time()-init_time)
            if new_Cost < oldCost:
                
                # Vẽ elip bằng cách xoay bề rộng và bề cao
                if graph.r1 != 0:
                    # ellipse_rect = pygame.Rect(0, 0, graph.r1 * 2, graph.r2 * 2)
                    # ellipse_rect.center = graph.center
                    # pygame.draw.ellipse(map_.map, (1,1,0), ellipse_rect, width=3)
                    # Tạo một surface riêng cho elip


                    ellipse_rect = pygame.Rect(0, 0, graph.r1 * 2, graph.r2 * 2)
                    ellipse_rect.center = (graph.r1, graph.r2)
                    ellipse_surface = pygame.Surface((graph.r1 * 2, graph.r2 * 2), pygame.SRCALPHA)
                    #ellipse_rect = ellipse_surface.get_rect(center=(graph.r1, graph.r2))
                    pygame.draw.ellipse(ellipse_surface, (255,0,0), ellipse_rect, width=3)
                    # Xoay elip
                    rotated_surface = pygame.transform.rotate(ellipse_surface, -graph.angle)
                    rotated_rect = rotated_surface.get_rect(center=graph.center)
                    graph.surface.blit(rotated_surface, rotated_rect.topleft)
                    pygame.display.update()  
                print("New Cost :" + str(new_Cost))

                if oldpath:
                    for i in range(0, len(oldpath) - 1):
                        pygame.draw.line(graph.surface, (255, 255, 255), oldpath[i], oldpath[i + 1],
                                         graph.edgeThickness + 5)
                        # time.sleep(0.03)
                        pygame.display.update()
                for i in range(0, len(graph.getPathcoords()) - 1):
                    pygame.draw.line(graph.surface, (255, 255, 0), graph.getPathcoords()[i], graph.getPathcoords()[i + 1],
                                     graph.edgeThickness + 5)
                    time.sleep(0.03)
                    pygame.display.update()
                oldCost = new_Cost
                oldpath = graph.getPathcoords()
                pygame.draw.circle(graph.surface, graph.green, graph.start, graph.nodeRadian + 10, 0)
                pygame.draw.circle(graph.surface, graph.red, graph.goal, graph.nodeRadian + 20, 1)
        # if iteration % 10 == 0:
        #     # Lưu màn hình Pygame vào file BMP
        #     pygame.image.save(map_.map, f"images/screenshot{n_image}.bmp")
        #
        #     # Sử dụng Pillow để mở và chuyển file BMP sang PNG hoặc GIF
        #     image = Image.open(f"images/screenshot{n_image}.bmp")
        #     image.save(f"images/screenshot{n_image}.png")  # Lưu dưới dạng PNG
        #     # image.save("screenshot.gif")  # Lưu dưới dạng GIF
        #     n_image += 1
    pygame.image.save(graph.surface, f"{graph.type}_algorithm.png")
    if graph.getfinalCost() != 0:

        print("Optimal Cost of " + str(iteration_max) + f" iterations is {graph.getfinalCost():.2f}")
    else:
        print("don't find any way to goal")
    eslaped = time.time()-start_time
    #eslaped = round(eslaped,2)
    print(f"\nTime of Searching:{eslaped:.2f} second\n")
    pygame.display.update()
    # pygame.event.clear()
    # pygame.event.wait(0)


if __name__ == '__main__':

    result = False
    pygame.init()
    # map_ = base_old.RRTmap(start, goal, dimensions, obsdim, obsnum, 'Informed RRT* Path Planning')
    graph_ = base_old.RRTgraph(start, goal, dimensions, obsdim, obsnum, type='RRT_star', Mapwindowname='Informed RRT* Path Planning')
    obs_ = graph_.makeObs()  # vật cản đã khuếch đại kích thước để tìm đường tránh va chạm của robot
    i_ = 0
    #print(obs_)
    graph_.drawmap(graph_.getTrueObs(obs_, adjust_obs))
    pygame.display.update()
    time.sleep(1)
    while not result:
        # if i_ > 0:
            # graph_.reset()
        try:
            main(obs_, graph_)
            if graph_.goalFlag:
                result = True
            else:
                print("finding path again...")
        except IndexError:
            result = False
        i_ += 1
        if i_ >= 3 and graph_.goalFlag:
            print("size of final path: " + str(len(graph_.getPathcoords())))
            print("final Cost is " + str(graph_.Cost_2))
            print("final Cost is " + str(graph_.getfinalCost()))
            break
        # if i_ > 20:
        #     print("error")
        #     break
        #     #     print("final Cost is " + str(graph_.getfinalCost()))
        # #     graph_.reset(all_=True)

    #khoi tao doi tuong RRT*
    map_1 = base_old.RRTmap(start, goal, dimensions, obsdim, obsnum, 'Informed RRT* Path Planning')
    graph_1 = base_old.RRTgraph(start, goal, dimensions, obsdim, obsnum, type='Informed_RRT_star')
    graph_1.obstacles=obs_
    result=False
    while not result:
        try:
            main(obs_, graph_1)
            if graph_1.goalFlag:
                result = True
            else:
                print("finding path again...")
        except IndexError:
            result = False
        i_ += 1
        if i_ >= 3 and graph_1.goalFlag:
            print("size of final path: " + str(len(graph_.getPathcoords())))
            print("final Cost is " + str(graph_.Cost_2))
            print("final Cost is " + str(graph_.getfinalCost()))
            break
    envir = car.Envir(dimensions)
    mobile_car = car.Robot(start, "car_img.png", graph_.waypoints2path())

    running = True
    dt = 0
    lasttime = pygame.time.get_ticks()
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        #     mobile_car.move(event)

        pygame.display.update()
        # envir.map.fill(envir.white)

        # envir.write_info(int(mobile_car.vl), int(mobile_car.vr), mobile_car.theta)

        dt = (pygame.time.get_ticks() - lasttime) / 1000
        lasttime = pygame.time.get_ticks()
        envir.map.fill(envir.white)
        graph_.drawmap(graph_.getTrueObs(obs_, adjust_obs))
        for coor in graph_.waypoints2path():
            pygame.draw.circle(graph_.surface, graph_.red, coor, graph_.nodeRadian, graph_.nodeThickness + 3)

        # i = len(graph_.getPathcoords())
        # pid_controller = car.PIDcontroller([car.Robot.x, graph_.y, graph_.theta], [target[0], target[1]])
        mobile_car.move(dt)
        mobile_car.draw(envir.map)
        # mobile_car.runPID(envir.map)
        # for coor in graph_.waypoints2path():
        #     pygame.draw.circle(map_.map, map_.red, coor, map_.nodeRadian, map_.nodeThickness + 3)

        envir.draw_trail((mobile_car.x, mobile_car.y))
        envir.draw_XYaxis_frame((mobile_car.x, mobile_car.y), mobile_car.theta)
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    ax.plot(time1, path_Cost1, linewidth=1, color='orange', label='RRT* path')
    ax.plot(time2, path_Cost2, linewidth=1, color='blue', label='Informed RRT* path')
    ax.set_title('Hiệu suất tìm kiếm đường đi của Informed RRT* và RRT*')
    ax.set_ylabel('Chi phí đường đi', fontsize=14)
    ax.set_xlabel('Thời gian(s)', fontsize=14)
    ax.tick_params(axis='both', labelsize=10)
    ax.legend()
    plt.savefig("InformedRRTandRRT_star_Performance.png")
    plt.show()