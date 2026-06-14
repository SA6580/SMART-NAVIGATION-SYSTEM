import java.util.*;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

// ==================== WAYPOINT CLASS ====================
class Waypoint {
    String id;
    String name;
    double longitude;
    double latitude;
    ArrayList<Waypoint> neighbors;

    public Waypoint(String id, String name, double latitude, double longitude) {
        this.id = id;
        this.name = name;
        this.latitude = latitude;
        this.longitude = longitude;
        this.neighbors = new ArrayList<>();
    }

    public double distanceTo(Waypoint other) {
        if (other == null) return Double.MAX_VALUE;
        double latDiff = this.latitude - other.latitude;
        double lonDiff = this.longitude - other.longitude;
        return Math.sqrt(latDiff * latDiff + lonDiff * lonDiff);
    }

    public void addNeighbor(Waypoint neighbor) {
        if (!neighbors.contains(neighbor)) {
            neighbors.add(neighbor);
        }
    }

    public ArrayList<Waypoint> getNeighbors() {
        return neighbors;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Waypoint waypoint = (Waypoint) obj;
        return id.equals(waypoint.id);
    }

    @Override
    public int hashCode() {
        return id.hashCode();
    }

    @Override
    public String toString() {
        return name + "(" + latitude + ", " + longitude + ")";
    }
}

// ==================== ROUTE CLASS ====================
class Route {
    LinkedList<Waypoint> path;
    double totalDistance;
    LocalDateTime timestamp;

    public Route(LinkedList<Waypoint> path, double totalDistance) {
        this.path = path;
        this.totalDistance = totalDistance;
        this.timestamp = LocalDateTime.now();
    }

    public void printRoute() {
        System.out.println("\n========== ROUTE DETAILS ==========");
        System.out.println("Total Distance: " + String.format("%.2f", totalDistance));
        System.out.println("Timestamp: " + timestamp.format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")));
        System.out.println("Number of Waypoints: " + path.size());
        System.out.print("Path: ");
        for (int i = 0; i < path.size(); i++) {
            System.out.print(path.get(i).name);
            if (i < path.size() - 1) System.out.print(" → ");
        }
        System.out.println("\n===================================\n");
    }

    public LinkedList<Waypoint> getPath() {
        return path;
    }

    public double getTotalDistance() {
        return totalDistance;
    }
}

// ==================== NAVIGATION REQUEST CLASS ====================
class NavigationRequest implements Comparable<NavigationRequest> {
    String startName;
    String endName;
    int priority;
    LocalDateTime requestTime;

    public NavigationRequest(String startName, String endName, int priority) {
        this.startName = startName;
        this.endName = endName;
        this.priority = priority;
        this.requestTime = LocalDateTime.now();
    }

    @Override
    public int compareTo(NavigationRequest other) {
        return Integer.compare(other.priority, this.priority);
    }

    @Override
    public String toString() {
        return startName + " -> " + endName + " (Priority: " + priority + ")";
    }
}

// ==================== WAYPOINT BST CLASS ====================
class WaypointBST {
    private Node root;

    private class Node {
        Waypoint waypoint;
        Node left;
        Node right;

        Node(Waypoint waypoint) {
            this.waypoint = waypoint;
        }
    }

    public void insert(Waypoint wp) {
        root = insertRec(root, wp);
    }

    private Node insertRec(Node node, Waypoint wp) {
        if (node == null) {
            return new Node(wp);
        }

        if (wp.longitude < node.waypoint.longitude) {
            node.left = insertRec(node.left, wp);
        } else if (wp.longitude > node.waypoint.longitude) {
            node.right = insertRec(node.right, wp);
        }
        return node;
    }

    public Waypoint search(double longitude) {
        return searchRec(root, longitude);
    }

    private Waypoint searchRec(Node node, double longitude) {
        if (node == null) {
            return null;
        }

        if (longitude < node.waypoint.longitude) {
            return searchRec(node.left, longitude);
        } else if (longitude > node.waypoint.longitude) {
            return searchRec(node.right, longitude);
        } else {
            return node.waypoint;
        }
    }

    public void inOrder() {
        inOrderRec(root);
    }

    private void inOrderRec(Node node) {
        if (node != null) {
            inOrderRec(node.left);
            System.out.print(node.waypoint.name + " ");
            inOrderRec(node.right);
        }
    }

    public int getHeight() {
        return getHeightRec(root);
    }

    private int getHeightRec(Node node) {
        if (node == null) return 0;
        return 1 + Math.max(getHeightRec(node.left), getHeightRec(node.right));
    }
}

// ==================== NAVIGATION SYSTEM CLASS ====================
class NavigationSystem {
    private ArrayList<Waypoint> waypoints;
    private HashMap<String, Waypoint> locationMap;
    private HashMap<String, Route> routeCache;
    private Stack<Route> navigationStack;
    private LinkedList<Route> navigationHistory;
    private PriorityQueue<NavigationRequest> requestQueue;
    private WaypointBST spatialIndex;
    private HashMap<String, Double> memoization;

    public NavigationSystem() {
        this.waypoints = new ArrayList<>();
        this.locationMap = new HashMap<>();
        this.routeCache = new HashMap<>();
        this.navigationStack = new Stack<>();
        this.navigationHistory = new LinkedList<>();
        this.requestQueue = new PriorityQueue<>();
        this.spatialIndex = new WaypointBST();
        this.memoization = new HashMap<>();
    }

    // ==================== ADD WAYPOINT ====================
    public void addWaypoint(String id, String name, double latitude, double longitude) {
        if (locationMap.containsKey(name)) {
            System.out.println("✗ Waypoint '" + name + "' already exists!");
            return;
        }
        Waypoint wp = new Waypoint(id, name, latitude, longitude);
        waypoints.add(wp);
        locationMap.put(name, wp);
        spatialIndex.insert(wp);
        System.out.println("✓ Waypoint added: " + name);
    }

    // ==================== CONNECT WAYPOINTS ====================
    public void connectWaypoints(String name1, String name2) {
        Waypoint wp1 = locationMap.get(name1);
        Waypoint wp2 = locationMap.get(name2);

        if (wp1 != null && wp2 != null) {
            wp1.addNeighbor(wp2);
            wp2.addNeighbor(wp1);
            System.out.println("✓ Connected: " + name1 + " ↔ " + name2);
        } else {
            System.out.println("✗ One or both waypoints not found!");
        }
    }

    // ==================== FIND SHORTEST PATH ====================
    private double findShortestPath(Waypoint current, Waypoint destination,
                                    HashMap<String, Double> memo, Set<Waypoint> visited) {
        if (current.equals(destination)) {
            return 0;
        }

        String key = current.id + "-" + destination.id;
        if (memo.containsKey(key)) {
            return memo.get(key);
        }

        if (visited.contains(current)) {
            return Double.MAX_VALUE;
        }
        visited.add(current);

        double minDistance = Double.MAX_VALUE;

        for (Waypoint neighbor : current.getNeighbors()) {
            if (!visited.contains(neighbor)) {
                double distance = current.distanceTo(neighbor) +
                        findShortestPath(neighbor, destination, memo, visited);
                minDistance = Math.min(minDistance, distance);
            }
        }

        visited.remove(current);
        memo.put(key, minDistance);
        return minDistance;
    }

    // ==================== BUILD ROUTE ====================
    private LinkedList<Waypoint> buildRoute(Waypoint current, Waypoint destination,
                                            HashMap<String, Double> memo, Set<Waypoint> visited) {
        LinkedList<Waypoint> route = new LinkedList<>();
        route.addFirst(current);

        if (current.equals(destination)) {
            return route;
        }

        if (visited.contains(current)) {
            return route;
        }
        visited.add(current);

        Waypoint bestNeighbor = null;
        double bestDistance = Double.MAX_VALUE;

        for (Waypoint neighbor : current.getNeighbors()) {
            if (!visited.contains(neighbor)) {
                String key = neighbor.id + "-" + destination.id;
                double distToDestination = memo.getOrDefault(key, Double.MAX_VALUE);
                double totalDist = current.distanceTo(neighbor) + distToDestination;

                if (totalDist < bestDistance) {
                    bestDistance = totalDist;
                    bestNeighbor = neighbor;
                }
            }
        }

        if (bestNeighbor != null) {
            LinkedList<Waypoint> restRoute = buildRoute(bestNeighbor, destination, memo, visited);
            route.addAll(restRoute);
        }

        visited.remove(current);
        return route;
    }

    // ==================== MAIN NAVIGATION METHOD ====================
    public Route navigateTo(String startName, String destName) {
        System.out.println("\n>>> Processing navigation request: " + startName + " → " + destName);

        String routeKey = startName + "-" + destName;
        if (routeCache.containsKey(routeKey)) {
            System.out.println("  ✓ Route found in cache!");
            return routeCache.get(routeKey);
        }

        Waypoint start = locationMap.get(startName);
        Waypoint destination = locationMap.get(destName);

        if (start == null || destination == null) {
            System.out.println("  ✗ Location not found!");
            return null;
        }

        System.out.println("  ⟳ Calculating shortest path...");
        HashMap<String, Double> memo = new HashMap<>();
        Set<Waypoint> visited = new HashSet<>();
        double distance = findShortestPath(start, destination, memo, visited);

        if (distance == Double.MAX_VALUE) {
            System.out.println("  ✗ No route found!");
            return null;
        }

        visited.clear();
        LinkedList<Waypoint> route = buildRoute(start, destination, memo, visited);

        Route result = new Route(route, distance);

        navigationStack.push(result);
        navigationHistory.addLast(result);

        routeCache.put(routeKey, result);
        System.out.println("  ✓ Route calculated and cached!");

        return result;
    }

    // ==================== UNDO NAVIGATION ====================
    public void undoNavigation() {
        if (!navigationStack.isEmpty()) {
            Route lastRoute = navigationStack.pop();
            System.out.println("\n✓ Undo: Removed route " + lastRoute.getPath().getFirst().name +
                    " → " + lastRoute.getPath().getLast().name);
        } else {
            System.out.println("\n✗ No history to undo!");
        }
    }

    // ==================== ADD REQUEST TO QUEUE ====================
    public void addNavigationRequest(String startName, String destName, int priority) {
        NavigationRequest request = new NavigationRequest(startName, destName, priority);
        requestQueue.offer(request);
        System.out.println("✓ Request queued: " + request);
    }

    // ==================== PROCESS QUEUE ====================
    public void processRequestQueue() {
        System.out.println("\n========== PROCESSING REQUEST QUEUE ==========");
        int count = 0;
        while (!requestQueue.isEmpty() && count < 10) {
            NavigationRequest request = requestQueue.poll();
            System.out.println("Processing: " + request);
            Route route = navigateTo(request.startName, request.endName);
            if (route != null) {
                route.printRoute();
            }
            count++;
        }
        System.out.println("==============================================\n");
    }

    // ==================== DISPLAY STATISTICS ====================
    public void displayStatistics() {
        System.out.println("\n========== SYSTEM STATISTICS ==========");
        System.out.println("Total Waypoints: " + waypoints.size());
        System.out.println("Cached Routes: " + routeCache.size());
        System.out.println("Navigation History Size: " + navigationHistory.size());
        System.out.println("Stack Size: " + navigationStack.size());
        System.out.println("Request Queue Size: " + requestQueue.size());
        System.out.println("BST Height: " + spatialIndex.getHeight());
        System.out.println("Memoization Cache Size: " + memoization.size());
        System.out.println("========================================\n");
    }

    // ==================== DISPLAY NAVIGATION HISTORY ====================
    public void displayNavigationHistory() {
        System.out.println("\n========== NAVIGATION HISTORY ==========");
        if (navigationHistory.isEmpty()) {
            System.out.println("No navigation history!");
        } else {
            int count = 1;
            for (Route route : navigationHistory) {
                System.out.print(count + ". ");
                for (int i = 0; i < route.getPath().size(); i++) {
                    System.out.print(route.getPath().get(i).name);
                    if (i < route.getPath().size() - 1) System.out.print(" → ");
                }
                System.out.println(" (Distance: " + String.format("%.2f", route.getTotalDistance()) + ")");
                count++;
            }
        }
        System.out.println("=========================================\n");
    }

    // ==================== DISPLAY ALL WAYPOINTS ====================
    public void displayAllWaypoints() {
        System.out.println("\n========== ALL WAYPOINTS ==========");
        if (waypoints.isEmpty()) {
            System.out.println("No waypoints added yet!");
        } else {
            for (Waypoint wp : waypoints) {
                System.out.println(wp.name + " at (" + wp.latitude + ", " + wp.longitude + ")");
                System.out.print("  Neighbors: ");
                if (wp.neighbors.isEmpty()) {
                    System.out.println("None");
                } else {
                    for (Waypoint neighbor : wp.neighbors) {
                        System.out.print(neighbor.name + " ");
                    }
                    System.out.println();
                }
            }
        }
        System.out.println("===================================\n");
    }

    public ArrayList<Waypoint> getAllWaypoints() {
        return waypoints;
    }
}

// ==================== MAIN CLASS WITH MENU ====================
public class SmartNavigationSystem {
    static NavigationSystem system = new NavigationSystem();
    static Scanner scanner = new Scanner(System.in);

    public static void main(String[] args) {
        System.out.println("\n╔════════════════════════════════════════╗");
        System.out.println("║   SMART NAVIGATION SYSTEM - PHASE 1    ║");
        System.out.println("║      WITH USER INPUT & INTERACTION     ║");
        System.out.println("╚════════════════════════════════════════╝\n");

        int choice;
        do {
            displayMainMenu();
            choice = getUserChoice();
            handleMainMenuChoice(choice);
        } while (choice != 0);

        System.out.println("\n╔════════════════════════════════════════╗");
        System.out.println("║   THANK YOU FOR USING THE SYSTEM       ║");
        System.out.println("╚════════════════════════════════════════╝\n");
        scanner.close();
    }

    // ==================== MAIN MENU ====================
    static void displayMainMenu() {
        System.out.println("\n╔════════════════════════════════════════╗");
        System.out.println("║          MAIN MENU                     ║");
        System.out.println("╠════════════════════════════════════════╣");
        System.out.println("║ 1. Add Waypoint                        ║");
        System.out.println("║ 2. Connect Waypoints                   ║");
        System.out.println("║ 3. Find Route (Navigation)             ║");
        System.out.println("║ 4. View All Waypoints                  ║");
        System.out.println("║ 5. View Navigation History             ║");
        System.out.println("║ 6. Add Navigation Request (Queue)      ║");
        System.out.println("║ 7. Process Request Queue               ║");
        System.out.println("║ 8. Undo Last Navigation                ║");
        System.out.println("║ 9. View System Statistics              ║");
        System.out.println("║ 10. Run Demo                           ║");
        System.out.println("║ 0. Exit                                ║");
        System.out.println("╚════════════════════════════════════════╝");
    }

    // ==================== GET USER CHOICE ====================
    static int getUserChoice() {
        System.out.print("Enter your choice: ");
        try {
            return scanner.nextInt();
        } catch (InputMismatchException e) {
            scanner.nextLine();
            System.out.println("Invalid input! Please enter a number.");
            return -1;
        }
    }

    // ==================== HANDLE MAIN MENU ====================
    static void handleMainMenuChoice(int choice) {
        switch (choice) {
            case 1:
                addWaypointMenu();
                break;
            case 2:
                connectWaypointsMenu();
                break;
            case 3:
                findRouteMenu();
                break;
            case 4:
                system.displayAllWaypoints();
                break;
            case 5:
                system.displayNavigationHistory();
                break;
            case 6:
                addRequestMenu();
                break;
            case 7:
                system.processRequestQueue();
                break;
            case 8:
                system.undoNavigation();
                break;
            case 9:
                system.displayStatistics();
                break;
            case 10:
                runDemo();
                break;
            case 0:
                System.out.println("Exiting...");
                break;
            default:
                System.out.println("Invalid choice! Please try again.");
        }
    }

    // ==================== ADD WAYPOINT MENU ====================
    static void addWaypointMenu() {
        System.out.println("\n╔════════════════════════════════════════╗");
        System.out.println("║        ADD WAYPOINT                    ║");
        System.out.println("╚════════════════════════════════════════╝");

        scanner.nextLine();  // Clear buffer

        System.out.print("Enter Waypoint ID (e.g., W1): ");
        String id = scanner.nextLine();

        System.out.print("Enter Waypoint Name (e.g., City Center): ");
        String name = scanner.nextLine();

        System.out.print("Enter Latitude: ");
        double latitude;
        try {
            latitude = scanner.nextDouble();
        } catch (InputMismatchException e) {
            System.out.println("Invalid latitude!");
            scanner.nextLine();
            return;
        }

        System.out.print("Enter Longitude: ");
        double longitude;
        try {
            longitude = scanner.nextDouble();
        } catch (InputMismatchException e) {
            System.out.println("Invalid longitude!");
            scanner.nextLine();
            return;
        }

        system.addWaypoint(id, name, latitude, longitude);
    }

    // ==================== CONNECT WAYPOINTS MENU ====================
    static void connectWaypointsMenu() {
        System.out.println("\n╔════════════════════════════════════════╗");
        System.out.println("║      CONNECT WAYPOINTS                 ║");
        System.out.println("╚════════════════════════════════════════╝");

        if (system.getAllWaypoints().isEmpty()) {
            System.out.println("No waypoints available! Add waypoints first.");
            return;
        }

        scanner.nextLine();  // Clear buffer

        System.out.print("Enter First Waypoint Name: ");
        String name1 = scanner.nextLine();

        System.out.print("Enter Second Waypoint Name: ");
        String name2 = scanner.nextLine();

        system.connectWaypoints(name1, name2);
    }

    // ==================== FIND ROUTE MENU ====================
    static void findRouteMenu() {
        System.out.println("\n╔════════════════════════════════════════╗");
        System.out.println("║        FIND ROUTE (NAVIGATE)           ║");
        System.out.println("╚════════════════════════════════════════╝");

        if (system.getAllWaypoints().isEmpty()) {
            System.out.println("No waypoints available! Add waypoints first.");
            return;
        }

        scanner.nextLine();  // Clear buffer

        System.out.print("Enter Start Location: ");
        String start = scanner.nextLine();

        System.out.print("Enter Destination Location: ");
        String destination = scanner.nextLine();

        Route route = system.navigateTo(start, destination);
        if (route != null) {
            route.printRoute();
        }
    }

    // ==================== ADD REQUEST MENU ====================
    static void addRequestMenu() {
        System.out.println("\n╔════════════════════════════════════════╗");
        System.out.println("║   ADD NAVIGATION REQUEST (QUEUE)       ║");
        System.out.println("╚════════════════════════════════════════╝");

        if (system.getAllWaypoints().isEmpty()) {
            System.out.println("No waypoints available! Add waypoints first.");
            return;
        }

        scanner.nextLine();  // Clear buffer

        System.out.print("Enter Start Location: ");
        String start = scanner.nextLine();

        System.out.print("Enter Destination Location: ");
        String destination = scanner.nextLine();

        System.out.println("Priority Levels: 1 (Normal) to 5 (Urgent)");
        System.out.print("Enter Priority (1-5): ");
        int priority;
        try {
            priority = scanner.nextInt();
            if (priority < 1 || priority > 5) {
                System.out.println("Priority must be between 1 and 5!");
                return;
            }
        } catch (InputMismatchException e) {
            System.out.println("Invalid priority!");
            scanner.nextLine();
            return;
        }

        system.addNavigationRequest(start, destination, priority);
    }

    // ==================== RUN DEMO ====================
    static void runDemo() {
        System.out.println("\n╔════════════════════════════════════════╗");
        System.out.println("║           RUNNING DEMO                 ║");
        System.out.println("╚════════════════════════════════════════╝");

        System.out.println("\n>>> Setting up city map with sample waypoints...\n");

        // Add waypoints
        system.addWaypoint("W1", "City Center", 10, 20);
        system.addWaypoint("W2", "Station", 15, 25);
        system.addWaypoint("W3", "Airport", 20, 30);
        system.addWaypoint("W4", "Hospital", 12, 18);
        system.addWaypoint("W5", "University", 18, 28);
        system.addWaypoint("W6", "Park", 22, 15);
        system.addWaypoint("W7", "Mall", 16, 22);

        // Connect waypoints
        System.out.println("\n>>> Connecting waypoints...\n");
        system.connectWaypoints("City Center", "Station");
        system.connectWaypoints("City Center", "Hospital");
        system.connectWaypoints("City Center", "Mall");
        system.connectWaypoints("Station", "Airport");
        system.connectWaypoints("Station", "University");
        system.connectWaypoints("Hospital", "Park");
        system.connectWaypoints("Mall", "University");
        system.connectWaypoints("Airport", "University");
        system.connectWaypoints("Park", "Airport");

        system.displayAllWaypoints();

        // Test navigation
        System.out.println(">>> Testing Navigation...\n");
        Route route1 = system.navigateTo("City Center", "Airport");
        if (route1 != null) route1.printRoute();

        Route route2 = system.navigateTo("Hospital", "University");
        if (route2 != null) route2.printRoute();

        // Test caching
        System.out.println(">>> Testing Cache (should be instant)...");
        Route route1_cached = system.navigateTo("City Center", "Airport");
        if (route1_cached != null) route1_cached.printRoute();

        // Test queue
        System.out.println("\n>>> Testing Priority Queue...\n");
        system.addNavigationRequest("City Center", "Park", 1);
        system.addNavigationRequest("Hospital", "Airport", 5);
        system.addNavigationRequest("Station", "Mall", 3);

        system.processRequestQueue();

        system.displayStatistics();
        system.displayNavigationHistory();
    }
}