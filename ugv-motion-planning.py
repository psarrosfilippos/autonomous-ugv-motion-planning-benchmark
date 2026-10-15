import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import heapq
import math
import random
from collections import defaultdict
import tkinter as tk
from tkinter import ttk
import sys

# ==========================================
# 1. ΠΕΡΙΒΑΛΛΟΝ, ΑΣΦΑΛΕΙΑ & ΕΛΕΓΧΟΣ ΚΕΝΟΥ 1.4x
# ==========================================

# Βοηθητική συνάρτηση για να μετατρέπουμε συντεταγμένες σε κελιά (scale 2x για ακρίβεια 0.5m)
def to_grid(x, y):
    return (int(round(x * 2.0)), int(round(y * 2.0)))

class RandomHiddenWorld:
    def __init__(self):
        # Αρχικοποιούμε τους τοίχους και τα εμπόδια του χάρτη
        self.walls = self._generate_random_map()
        self.hidden_obs = self._generate_points()
        
    def _get_segment_points(self, segment):
        # Παίρνουμε όλα τα ενδιάμεσα διακριτά σημεία μιας ευθείας (για τα εμπόδια)
        p1, p2 = segment
        steps = int(math.hypot(p2[0]-p1[0], p2[1]-p1[1]) * 2.0)
        pts = []
        for i in range(steps + 1):
            x = p1[0] + (p2[0] - p1[0]) * i / max(steps, 1)
            y = p1[1] + (p2[1] - p1[1]) * i / max(steps, 1)
            pts.append((x, y))
        return pts
        
    def _generate_random_map(self):
        # Βασικό περίγραμμα (τα σύνορα του χάρτη)
        walls = [[(2, 2), (50, 2)], [(50, 2), (50, 30)], [(50, 30), (2, 30)], [(2, 30), (2, 2)]]
        
        # Λίστα για να ελέγχουμε αποστάσεις μεταξύ εμποδίων
        existing_points = []
        for w in walls:
            existing_points.extend(self._get_segment_points(w))
            
        attempts = 0
        added_walls = 0
        
        # Ρίχνουμε τυχαία εμπόδια μέχρι να φτάσουμε τα 12 ή αν κολλήσει μετά από 200 προσπάθειες
        while added_walls < 12 and attempts < 200:
            attempts += 1
            x = random.randint(8, 42)
            y = random.randint(6, 24)
            length = random.randint(4, 9)
            is_horiz = random.random() > 0.5
            wx2, wy2 = (x + length, y) if is_horiz else (x, y + length)
            
            # Έλεγχος: Κρατάμε τα εμπόδια μακριά από αφετηρία και στόχο για να μην τα εγκλωβίσουν
            dist_start = min(math.hypot(x-4, y-4), math.hypot(wx2-4, wy2-4))
            dist_goal = min(math.hypot(x-48, y-28), math.hypot(wx2-48, wy2-28))
            
            if dist_start < 8.0 or dist_goal < 8.0:
                continue
                
            # Έλεγχος κενού: Πρέπει να έχει απόσταση τουλάχιστον 5m (1.4x το πλάτος) από άλλα εμπόδια
            cand_points = self._get_segment_points([(x, y), (wx2, wy2)])
            too_close = False
            for cx, cy in cand_points:
                for ex, ey in existing_points:
                    if math.hypot(cx - ex, cy - ey) < 5.0:
                        too_close = True
                        break
                if too_close: break
            
            # Αν περάσει τους ελέγχους, το προσθέτουμε στο χάρτη
            if not too_close:
                walls.append([(x, y), (wx2, wy2)])
                existing_points.extend(cand_points)
                added_walls += 1
                
        return walls

    def _generate_points(self):
        # Επιστρέφει όλα τα σημεία εμποδίων μαζεμένα
        obs = []
        for w in self.walls:
            obs.extend(self._get_segment_points(w))
        return obs

def scan_env(pos, radius, hidden_obs, known_obs, inflated_grid):
    # Προσομοίωση Lidar: Βρίσκει κρυφά εμπόδια σε μια ακτίνα γύρω από το όχημα
    new_found = []
    for ox, oy in hidden_obs:
        if (ox, oy) not in known_obs:
            # Αρχικά γρήγορο bounding box check, μετά κανονική ευκλείδεια απόσταση
            if abs(ox - pos[0]) <= radius and abs(oy - pos[1]) <= radius:
                if math.hypot(ox - pos[0], oy - pos[1]) <= radius:
                    known_obs.add((ox, oy))
                    new_found.append((ox, oy))
                    
                    # Κάνουμε "inflate" (φούσκωμα) τα εμπόδια κατά 2.2m για ασφάλεια του οχήματος
                    for dx in [-2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0]:
                        for dy in [-2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0]:
                            if math.hypot(dx, dy) <= 2.2: 
                                inflated_grid.add(to_grid(ox + dx, oy + dy))
    return new_found

def is_blocked(nx, ny, curr_x, curr_y, inflated_grid):
    # Ελέγχει αν το επόμενο σημείο (ή το ενδιάμεσο του) πέφτει πάνω σε φουσκωμένο εμπόδιο
    if to_grid(nx, ny) in inflated_grid: return True
    mid_x, mid_y = (nx + curr_x)/2.0, (ny + curr_y)/2.0
    if to_grid(mid_x, mid_y) in inflated_grid: return True
    return False

def is_path_blocked(path, inflated_grid):
    # Ελέγχει αν έχει μπλοκαριστεί κάποιο σημείο στο ήδη χαραγμένο μονοπάτι
    if not path or len(path) <= 1: return True
    for i in range(len(path)-1):
        if is_blocked(path[i+1][0], path[i+1][1], path[i][0], path[i][1], inflated_grid):
            return True
    return False

def get_car_lines(x, y, yaw):
    # Υπολογισμός των 4 γωνιών του οχήματος (3.5 x 1.8) για να το ζωγραφίσουμε σωστά στο γράφημα
    length, width = 3.5, 1.8
    cx = [-length/2, length/2, length/2, -length/2, -length/2]
    cy = [width/2, width/2, -width/2, -width/2, width/2]
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rot = np.dot(R, np.array([cx, cy]))
    return rot[0, :] + x, rot[1, :] + y

def get_blind_wander_target(pos, inflated_grid, heatmap, start_pos):
    # Κλιμακωτός έλεγχος αδράνειας: Δοκιμάζει να συνεχίσει ευθεία αν έχει κενό μπροστά (25m, 15m κλπ)
    for dist in [25.0, 15.0, 10.0, 5.0, 3.0]:
        rx_s = pos[0] + dist * math.cos(pos[2])
        ry_s = pos[1] + dist * math.sin(pos[2])
        if 4.0 <= rx_s <= 48.0 and 4.0 <= ry_s <= 28.0:
            if to_grid(rx_s, ry_s) not in inflated_grid and heatmap[to_grid(rx_s, ry_s)] < 50:
                return (rx_s, ry_s) # Συνεχίζει ευθεία, δεν στρίβει
            
    # Αν έχει τοίχο μπροστά, ψάχνει τυχαίο σημείο μακριά από αφετηρία και καμένες περιοχές (heatmap)
    best_target = None
    max_score = -float('inf')
    for _ in range(50):
        rx = random.uniform(5, 47)
        ry = random.uniform(5, 27)
        g_pt = to_grid(rx, ry)
        
        if g_pt not in inflated_grid:
            score = math.hypot(rx - start_pos[0], ry - start_pos[1]) + \
                    math.hypot(rx - pos[0], ry - pos[1]) - \
                    (heatmap[g_pt] * 200.0) # Τεράστια ποινή στα μέρη που έχουμε ξαναπάει
            
            if score > max_score:
                max_score = score
                best_target = (rx, ry)
    return best_target if best_target else (25.0, 15.0)

# ==========================================
# 2. ΑΛΓΟΡΙΘΜΟΙ
# ==========================================
def reconstruct_path(curr, came_from):
    # Ξετυλίγουμε το λεξικό για να φτιάξουμε τη λίστα της διαδρομής
    path = []
    while curr: path.append(curr); curr = came_from.get(curr, None)
    path = path[::-1] # Το αντιστρέφουμε γιατί ξεκινάει από το τέλος
    
    # Υπολογίζουμε τις γωνίες πορείας (yaw) μεταξύ των σημείων
    final_path = []
    for i in range(len(path)-1):
        yaw = math.atan2(path[i+1][1]-path[i][1], path[i+1][0]-path[i][0])
        final_path.append((path[i][0], path[i][1], yaw))
    if final_path: final_path.append((path[-1][0], path[-1][1], final_path[-1][2]))
    return final_path

def plan_astar(start, goal, inflated_grid, heatmap):
    # Κλασικός A* (σε grid 8 κατευθύνσεων)
    start_node = (round(start[0]), round(start[1]))
    open_set = []
    heapq.heappush(open_set, (0, start_node, (1, 0))) # Το (1,0) είναι η αρχική κατεύθυνση
    came_from, g_score = {start_node: None}, {start_node: 0}
    
    while open_set:
        _, curr, prev_d = heapq.heappop(open_set)
        
        # Αν φτάσαμε κοντά (2m), ολοκληρώνουμε την αναζήτηση
        if math.hypot(curr[0]-goal[0], curr[1]-goal[1]) < 2.0:
            return reconstruct_path(curr, came_from)

        for dx, dy in [(1,0), (1,1), (0,1), (-1,1), (-1,0), (-1,-1), (0,-1), (1,-1)]:
            nx, ny = curr[0]+dx, curr[1]+dy
            if is_blocked(nx, ny, curr[0], curr[1], inflated_grid): continue
            
            # Κανόνας Δεξιού Τοίχου: Βάζουμε ποινή 2.0 αν το όχημα στρίψει αριστερά
            cross = prev_d[0]*dy - prev_d[1]*dx
            turn_penalty = 2.0 if cross > 0 else 0.0 
            
            # Ποινή Μνήμης (για να αποφεύγουμε τις λούπες)
            memory_penalty = heatmap[to_grid(nx, ny)] * 1.5 
            cost = g_score[curr] + math.hypot(dx, dy) + memory_penalty + turn_penalty
            
            if (nx, ny) not in g_score or cost < g_score[(nx, ny)]:
                g_score[(nx, ny)] = cost
                h = math.hypot(nx-goal[0], ny-goal[1]) * 1.0 # Heuristic απόσταση
                heapq.heappush(open_set, (cost + h, (nx, ny), (dx, dy)))
                came_from[(nx, ny)] = curr
    return []

def plan_rrt(start, goal, inflated_grid, heatmap):
    # Αλγόριθμος RRT (Rapidly-exploring Random Tree)
    class Node:
        def __init__(self, x, y, yaw): self.x, self.y, self.yaw, self.p = x, y, yaw, None
    nodes = [Node(start[0], start[1], start[2])]
    
    closest_ind = 0
    min_dist = math.hypot(start[0]-goal[0], start[1]-goal[1])
    
    for _ in range(3000): # Μέγιστες προσπάθειες δέντρου
        # Στοχεύει απευθείας στο goal με 15% πιθανότητα (Bias)
        rnd = goal if random.random() > 0.15 else [random.uniform(2,50), random.uniform(2,30)]
        n_ind = min(range(len(nodes)), key=lambda i: math.hypot(nodes[i].x-rnd[0], nodes[i].y-rnd[1]))
        near = nodes[n_ind]
        
        # Υπολογισμός γωνίας προς το τυχαίο σημείο
        theta = math.atan2(rnd[1]-near.y, rnd[0]-near.x)
        yaw_diff = (theta-near.yaw+math.pi)%(2*math.pi)-math.pi
        
        direction = 1.0
        if abs(yaw_diff) > math.pi/2:
            direction = -1.0 # Επιτρέπει οπισθοπορεία αν χρειαστεί
            yaw_diff = (theta-(near.yaw+math.pi)+math.pi)%(2*math.pi)-math.pi
            
        # Ελαφρύτερη ποινή αριστερής στροφής (για να μην κολλάει ο RRT)
        if yaw_diff > 0.1: 
            yaw_diff *= 0.5 
            
        # Περιορίζουμε την γωνία στροφής τιμονιού
        yaw_diff = max(min(yaw_diff, 0.4), -0.4)
        new_yaw = near.yaw + yaw_diff
        nx = near.x + direction * 2.0 * math.cos(new_yaw)
        ny = near.y + direction * 2.0 * math.sin(new_yaw)
        
        if not is_blocked(nx, ny, near.x, near.y, inflated_grid):
            # Αποφεύγει πολύ καμένα σημεία του heatmap
            if heatmap[to_grid(nx, ny)] > 5 and random.random() < 0.6: continue 
                
            new_node = Node(nx, ny, new_yaw)
            new_node.p = n_ind
            nodes.append(new_node)
            
            # Ενημέρωση κοντινότερου σημείου για το Failsafe
            dist = math.hypot(nx-goal[0], ny-goal[1])
            if dist < min_dist:
                min_dist = dist
                closest_ind = len(nodes)-1
            
            # Αν έφτασε, τέλος
            if dist < 2.5:
                path, curr = [], len(nodes)-1
                while curr is not None:
                    n = nodes[curr]
                    path.append((n.x, n.y, n.yaw))
                    curr = n.p
                return path[::-1]
                
    # Αν τερματίσει χωρίς να φτάσει, επιστρέφει το καλύτερο μονοπάτι που βρήκε (RRT Fallback)
    path, curr = [], closest_ind
    while curr is not None:
        n = nodes[curr]
        path.append((n.x, n.y, n.yaw))
        curr = n.p
    return path[::-1] if len(path) > 1 else []

def plan_hybrid(start, goal, inflated_grid, heatmap):
    # Hybrid A*: Λειτουργεί με συνεχή κινηματική, αλλά αποθηκεύει σε grid
    def get_key(c, direction): return (int(round(c[0])), int(round(c[1])), int(round(c[2]*2.0)), direction)
    open_set = []
    heapq.heappush(open_set, (0, start, 1.0))
    start_key = get_key(start, 1.0)
    came_from_raw = {(start[0], start[1]): None} 
    g_score = {start_key: 0}

    while open_set:
        _, curr, curr_dir = heapq.heappop(open_set)
        curr_k = get_key(curr, curr_dir)
        
        if math.hypot(curr[0]-goal[0], curr[1]-goal[1]) < 2.0:
            return reconstruct_path((curr[0], curr[1]), came_from_raw)

        # Δοκιμάζουμε εμπρός και πίσω κίνηση
        for direction in [1.0, -1.0]:
            # 3 γωνίες τιμονιού (Δεξιά, Κέντρο, Αριστερά)
            for angle in [-0.5, 0, 0.5]:
                nyaw = curr[2] + direction * angle
                nx = curr[0] + direction * 1.5 * math.cos(nyaw)
                ny = curr[1] + direction * 1.5 * math.sin(nyaw)
                
                if is_blocked(nx, ny, curr[0], curr[1], inflated_grid): continue
                
                neighbor = (nx, ny, nyaw)
                n_k = get_key(neighbor, direction)
                
                # Ποινή για αριστερή στροφή
                turn_penalty = 3.0 if angle > 0 else 0.0 
                memory_penalty = heatmap[to_grid(nx, ny)] * 1.0 
                
                cost = g_score[curr_k] + 1.5 + turn_penalty + memory_penalty
                
                if n_k not in g_score or cost < g_score[n_k]:
                    g_score[n_k] = cost
                    h = math.hypot(nx-goal[0], ny-goal[1]) * 1.0 
                    heapq.heappush(open_set, (cost + h, neighbor, direction))
                    if (nx, ny) not in came_from_raw:
                        came_from_raw[(nx, ny)] = (curr[0], curr[1])
    return []

# ==========================================
# 3. ΕΚΤΕΛΕΣΗ ΜΕ UI ΕΠΑΝΑΛΗΨΗΣ (RETRY)
# ==========================================
def run_simulation_process(algo_func, initial_knows_goal, algo_name, root_window):
    # Μπαίνουμε σε While-True λούπα για να παίζει το κουμπί 'Επανάληψη'
    while True: 
        world = RandomHiddenWorld()
        actual_goal = (48.0, 28.0)
        start_pos = [4.0, 4.0, np.pi/4]
        sensor_radius = 8.0
        
        # Καθαρίζουμε εντελώς τη μνήμη (heatmap κλπ) για το νέο γύρο
        known = set()
        inflated = set()
        heatmap = defaultdict(int) 
        knows_goal = initial_knows_goal
        
        target = actual_goal if knows_goal else get_blind_wander_target(start_pos, inflated, heatmap, start_pos)
        path = []
        
        mode = "NORMAL"
        failsafe_strikes = 0 
        driven_history = [start_pos]
        
        sim_state = {"action": "RUNNING"} 
        
        # Στήσιμο Matplotlib γραφικών
        fig, ax = plt.subplots(figsize=(12, 8))
        plt.subplots_adjust(bottom=0.2) 
        fig.canvas.manager.set_window_title(f'Εκτέλεση: {algo_name}')
        ax.set_xlim(0, 52); ax.set_ylim(0, 32); ax.set_aspect('equal'); ax.grid(True)
        ax.set_title(f"{algo_name} | {'Γνωστός Στόχος' if knows_goal else 'Τυφλή Εξερεύνηση'}")
        
        # Ζωγραφίζουμε τους τοίχους και το αρχικό όχημα
        for p1, p2 in world.walls: 
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='lightgray', linewidth=4, alpha=0.5)
            
        ax.plot(start_pos[0], start_pos[1], 'rs', markersize=8)
        goal_color = 'g*' if knows_goal else 'k*'
        goal_alpha = 1.0 if knows_goal else 0.3
        goal_s, = ax.plot(actual_goal[0], actual_goal[1], goal_color, markersize=12, alpha=goal_alpha)
        
        # Γραμμές για το αμάξι, το μονοπάτι, και τον αισθητήρα (Lidar)
        car_l, = ax.plot([], [], 'r-', linewidth=2)
        plan_l, = ax.plot([], [], 'blue', linestyle='--', alpha=0.5)
        trail_l, = ax.plot([], [], 'blue', linewidth=2)
        disc_s, = ax.plot([], [], 'ko', markersize=2)
        sensor = plt.Circle((0,0), sensor_radius, color='orange', fill=False, linestyle=':')
        ax.add_patch(sensor)
        
        # --- Κουμπιά UI (Κατά την εκτέλεση) ---
        ax_stop = plt.axes([0.25, 0.05, 0.2, 0.075])
        btn_stop = Button(ax_stop, 'Τερματισμός / Μενού', color='salmon', hovercolor='red')
        def stop_cb(event): sim_state["action"] = "MENU"
        btn_stop.on_clicked(stop_cb)
        
        ax_retry = plt.axes([0.55, 0.05, 0.2, 0.075])
        btn_retry = Button(ax_retry, 'Επανάληψη', color='lightblue', hovercolor='skyblue')
        def retry_cb(event): sim_state["action"] = "RETRY"
        btn_retry.on_clicked(retry_cb)

        tx, ty, dx, dy = [], [], [], []
        pos = list(start_pos)
        
        reached_goal = False
        
        # Η κύρια λούπα κίνησης
        while sim_state["action"] == "RUNNING":
            # Έλεγχος αν πιάσαμε τον τελικό στόχο
            if knows_goal and math.hypot(pos[0]-actual_goal[0], pos[1]-actual_goal[1]) < 2.0:
                reached_goal = True
                ax.set_title("ΟΛΟΚΛΗΡΩΣΗ! Ο στόχος βρέθηκε.")
                plan_l.set_data([], [])
                break
                
            # Ενημέρωση heatmap διέλευσης
            heatmap[to_grid(pos[0], pos[1])] += 1 
                
            # Λήψη δεδομένων από αισθητήρα
            new_obs = scan_env(pos, sensor_radius, world.hidden_obs, known, inflated)
            if new_obs:
                dx.extend([p[0] for p in new_obs]); dy.extend([p[1] for p in new_obs])
                disc_s.set_data(dx, dy)
            
            # Λογική τυφλής εξερεύνησης
            if not knows_goal:
                if math.hypot(pos[0]-actual_goal[0], pos[1]-actual_goal[1]) <= sensor_radius:
                    # Είδαμε το στόχο, σταματάμε να ψάχνουμε τυφλά
                    knows_goal = True
                    target = actual_goal
                    path = []
                    goal_s.set_color('g'); goal_s.set_alpha(1.0)
                elif len(path) < 10 and mode == "NORMAL": 
                    # Κυλιόμενη ανανέωση στόχου εν κινήσει
                    target = get_blind_wander_target(pos, inflated, heatmap, start_pos)
                    path = [] 
                    
            # Αν έχουμε μπλοκαριστεί ή δεν έχουμε path, υπολογίζουμε νέο
            if is_path_blocked(path, inflated) or len(path) <= 1:
                if mode != "NORMAL":
                    # Όταν είμαστε σε Failsafe, χρησιμοποιούμε πάντα A* για σιγουριά
                    path = plan_astar(pos, target, inflated, heatmap)
                    
                    # INFINITE LOOP FIX: Αν το Failsafe δεν βρίσκει ούτε αυτό δρόμο
                    if len(path) <= 1:
                        if mode == "FAILSAFE_1":
                            print(">> Failsafe 1: Εγκλωβισμός! Κλιμάκωση σε Failsafe+...")
                            mode = "FAILSAFE_PLUS"
                            target = start_pos # Γυρνάμε αρχή
                            plan_l.set_color('red')
                            path = plan_astar(pos, target, inflated, heatmap)
                            
                        if len(path) <= 1 and mode == "FAILSAFE_PLUS":
                            print(">> Failsafe+: Κρίσιμος Εγκλωβισμός! Καθαρισμός τοπικής μνήμης (Unstuck)...")
                            # Σβήνει τη μνήμη γύρω του για να μπορέσει να δραπετεύσει ελεύθερα
                            for rx in range(-8, 9):
                                for ry in range(-8, 9):
                                    heatmap[to_grid(pos[0]+rx, pos[1]+ry)] = 0
                            path = plan_astar(pos, target, inflated, heatmap)
                else:
                    # Κανονικός υπολογισμός με τον αλγόριθμο που επέλεξε ο χρήστης
                    path = algo_func(pos, target, inflated, heatmap)
                    
                if len(path) <= 1 and mode == "NORMAL":
                    # MICRO-PUSH: Πριν τα παρατήσουμε, σκάβουμε μπροστά 1.5 μέτρο
                    push_x = pos[0] + 1.5 * math.cos(pos[2])
                    push_y = pos[1] + 1.5 * math.sin(pos[2])
                    
                    # FIX: Διόρθωση ονόματος σε 'inflated' για να μη σκάει Error
                    if not is_blocked(push_x, push_y, pos[0], pos[1], inflated) and (4.0 <= push_x <= 48.0) and (4.0 <= push_y <= 28.0):
                        path = [
                            (pos[0], pos[1], pos[2]),
                            (push_x, push_y, pos[2])
                        ]
                    else:
                        # Το Micro-Push απέτυχε, άρα όντως κολλήσαμε. Ενεργοποιούμε Failsafe.
                        failsafe_strikes += 1
                        
                        # Καίμε έντονα την περιοχή γύρω μας για να μην ξαναγυρίσουμε
                        for rx in range(-6, 7):
                            for ry in range(-6, 7):
                                heatmap[to_grid(pos[0]+rx, pos[1]+ry)] += 100
                        
                        if failsafe_strikes <= 2:
                            # Failsafe 1: Κάνουμε πίσω 25 βήματα
                            mode = "FAILSAFE_1"
                            back_idx = max(0, len(driven_history) - 25)
                            target = driven_history[back_idx]
                            plan_l.set_color('orange')
                        else:
                            # Failsafe+: Πάμε πάλι στην αρχή
                            mode = "FAILSAFE_PLUS"
                            target = start_pos
                            failsafe_strikes = 0 
                            plan_l.set_color('red')
                        
                        path = plan_astar(pos, target, inflated, heatmap)
                        
            # Επιστροφή στην κανονικότητα αν πιάσαμε τον Failsafe στόχο
            if mode != "NORMAL" and math.hypot(pos[0]-target[0], pos[1]-target[1]) < 2.5:
                mode = "NORMAL"
                plan_l.set_color('blue')
                target = actual_goal if knows_goal else get_blind_wander_target(pos, inflated, heatmap, start_pos)
                path = []

            # Κίνηση του οχήματος
            if len(path) > 1:
                plan_l.set_data([p[0] for p in path], [p[1] for p in path])
                next_step = path.pop(1)
                pos = [next_step[0], next_step[1], next_step[2]]
                driven_history.append(pos) # Αποθήκευση ιστορικού κίνησης
            else:
                plan_l.set_data([], [])
                    
            # Ενημέρωση γραφικών (ίχνος, αμάξι, αισθητήρας)
            tx.append(pos[0]); ty.append(pos[1])
            trail_l.set_data(tx, ty)
            
            cx, cy = get_car_lines(pos[0], pos[1], pos[2])
            car_l.set_data(cx, cy)
            sensor.center = (pos[0], pos[1])
            
            plt.pause(0.015)

        # --- ΚΟΥΜΠΙΑ ΟΤΑΝ ΟΛΟΚΛΗΡΩΘΕΙ ΕΠΙΤΥΧΩΣ ---
        # Κρύβουμε τα παλιά κουμπιά και εμφανίζουμε το τελικό μενού
        if reached_goal and sim_state["action"] == "RUNNING":
            sim_state["action"] = "DONE"
            ax_stop.set_visible(False)
            ax_retry.set_visible(False)
            
            ax_ret = plt.axes([0.15, 0.05, 0.2, 0.075])
            btn_ret = Button(ax_ret, 'Μενού', color='lightgreen')
            
            ax_retry_end = plt.axes([0.40, 0.05, 0.2, 0.075])
            btn_retry_end = Button(ax_retry_end, 'Επανάληψη', color='lightblue')
            
            ax_exit = plt.axes([0.65, 0.05, 0.2, 0.075])
            btn_exit = Button(ax_exit, 'Έξοδος', color='salmon')
            
            def ret_cb(event): sim_state["action"] = "MENU"
            def rend_cb(event): sim_state["action"] = "RETRY"
            def exit_cb(event): sim_state["action"] = "EXIT"
            
            btn_ret.on_clicked(ret_cb)
            btn_retry_end.on_clicked(rend_cb)
            btn_exit.on_clicked(exit_cb)
            
            # Περιμένουμε την επιλογή του χρήστη
            while sim_state["action"] == "DONE" and plt.fignum_exists(fig.number):
                plt.pause(0.1)

        plt.close(fig)
        
        # Λογική ελέγχου επιλογής μετά τον τερματισμό
        if sim_state["action"] == "RETRY":
            continue # Πάμε στην αρχή της while True (Νέος χάρτης)
        elif sim_state["action"] == "EXIT":
            sys.exit(0)
        else:
            root_window.deiconify() # Γυρνάμε στο κεντρικό UI
            break

# ==========================================
# 4. ΓΡΑΦΙΚΟ ΚΕΝΤΡΙΚΟ ΜΕΝΟΥ (TKINTER)
# ==========================================
class AppMenu:
    def __init__(self):
        # Χτίζω το κεντρικό παράθυρο UI
        self.root = tk.Tk()
        self.root.title("Σύστημα Αυτόνομης Πλοήγησης")
        self.root.geometry("450x550")
        self.root.configure(bg="#2c3e50")
        
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TButton", font=("Segoe UI", 11, "bold"), padding=10, background="#ecf0f1")
        
        title_label = tk.Label(self.root, text="ΕΠΙΛΟΓΗ ΑΛΓΟΡΙΘΜΟΥ", font=("Segoe UI", 16, "bold"), bg="#2c3e50", fg="white")
        title_label.pack(pady=20)
        
        # Λίστα με τις επιλογές για τα κουμπιά
        self.algorithms = [
            ("1. A* (Γνωστός Στόχος)", plan_astar, True),
            ("2. RRT (Γνωστός Στόχος)", plan_rrt, True),
            ("3. Hybrid (Γνωστός Στόχος)", plan_hybrid, True),
            ("4. A* (Τυφλή Εξερεύνηση)", plan_astar, False),
            ("5. RRT (Τυφλή Εξερεύνηση)", plan_rrt, False),
            ("6. Hybrid (Τυφλή Εξερεύνηση)", plan_hybrid, False)
        ]
        
        # Δημιουργία των κουμπιών αλγορίθμων
        for name, func, knows_goal in self.algorithms:
            btn = ttk.Button(self.root, text=name, command=lambda f=func, k=knows_goal, n=name: self.start_sim(f, k, n))
            btn.pack(fill='x', padx=40, pady=8)
            
        # Κουμπί Έξοδος
        style.configure("Exit.TButton", background="#e74c3c", foreground="white")
        exit_btn = ttk.Button(self.root, text="Έξοδος", style="Exit.TButton", command=self.root.quit)
        exit_btn.pack(fill='x', padx=40, pady=30)
        
        info_label = tk.Label(self.root, text="Σύστημα πλοήγησης οχήματος σε άγνωστο περιβάλλον.", font=("Segoe UI", 9), bg="#2c3e50", fg="#bdc3c7")
        info_label.pack(side='bottom', pady=10)

    def start_sim(self, func, knows_goal, name):
        # Κρύβει το UI και ξεκινάει τη matplotlib προσομοίωση
        self.root.withdraw() 
        run_simulation_process(func, knows_goal, name, self.root)

if __name__ == '__main__':
    app = AppMenu()
    app.root.mainloop()