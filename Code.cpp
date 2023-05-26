#include <iostream>
#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <queue>

using namespace std;
class Queue
{
private:
    static const int MAX_SIZE = 100;
public:
    string queue[MAX_SIZE];
    int front;
    int rear;

    Queue()
    {
        front = -1;
        rear = -1;
    }

    bool isEmptyQ()
    {
        return front == -1;
    }

    bool isFull()
    {
        return (rear + 1) % MAX_SIZE == front;
    }

    void enqueue(string item)
    {
        if (isFull())
        {
            cout << "Queue is full. Cannot enqueue item." << endl;
            return;
        }
        if (isEmptyQ())
        {
            front = 0;
        }
        rear = (rear + 1) % MAX_SIZE;
        queue[rear] = item;
    }

    string dequeue()
    {
        if (isEmptyQ())
        {
            cout << "Queue is empty. Cannot dequeue item." << endl;
            return "";
        }
        string item = queue[front];
        if (front == rear)
        {
            front = -1;
            rear = -1;
        }
        else
        {
            front = (front + 1) % MAX_SIZE;
        }
        return item;
    }

    int size()
    {
        if (isEmptyQ())
        {
            return 0;
        }
        if (front <= rear)
        {
            return rear - front + 1;
        }
        else
        {
            return MAX_SIZE - front + rear + 1;
        }
    }
};
const int MAX = 100;
class MyStack
{
public:
    int top;
    string arr[MAX];

    MyStack()
    {
        top = -1;
    }

    void push(string x)
    {
        if (top >= MAX - 1)
        {
            cout << "Stack Overflow" << endl;
        }
        else
        {
            top++;
            arr[top] = x;
        }
    }

    void pop()
    {
        if (top < 0)
        {
            cout << "Stack Underflow" << endl;
        }
        else
        {
            string x = arr[top];
            top--;
        }
    }

    bool isEmpty()
    {
        return top < 0;
    }
};

struct Node
{
    string name;
    int cost;

    Node(const string& n, int c) : name(n), cost(c) {}

    // Custom comparison operator for priority queue
    bool operator >(const Node& other) const
    {
        return cost > other.cost;
    }
};
using Graph = map<string, vector<pair<string, int>>>;

Graph romaniaMap =
{
    {"Arad", {{"Sibiu", 140}, {"Zerind", 75}, {"Timisoara", 118}}},
    {"Zerind", {{"Arad", 75}, {"Oradea", 71}}},
    {"Oradea", {{"Zerind", 71}, {"Sibiu", 151}}},
    {"Sibiu", {{"Arad", 140}, {"Oradea", 151}, {"Fagaras", 99}, {"Rimnicu", 80}}},
    {"Timisoara", {{"Arad", 118}, {"Lugoj", 111}}},
    {"Lugoj", {{"Timisoara", 111}, {"Mehadia", 70}}},
    {"Mehadia", {{"Lugoj", 70}, {"Drobeta", 75}}},
    {"Drobeta", {{"Mehadia", 75}, {"Craiova", 120}}},
    {"Craiova", {{"Drobeta", 120}, {"Rimnicu", 146}, {"Pitesti", 138}}},
    {"Rimnicu", {{"Sibiu", 80}, {"Craiova", 146}, {"Pitesti", 97}}},
    {"Fagaras", {{"Sibiu", 99}, {"Bucharest", 211}}},
    {"Pitesti", {{"Rimnicu", 97}, {"Craiova", 138}, {"Bucharest", 101}}},
    {"Bucharest", {{"Fagaras", 211}, {"Pitesti", 101}, {"Giurgiu", 90}, {"Urziceni", 85}}},
    {"Giurgiu", {{"Bucharest", 90}}},
    {"Urziceni", {{"Bucharest", 85}, {"Vaslui", 142}, {"Hirsova", 98}}},
    {"Hirsova", {{"Urziceni", 98}, {"Eforie", 86}}},
    {"Eforie", {{"Hirsova", 86}}},
    {"Vaslui", {{"Urziceni", 142}, {"Iasi", 92}}},
    {"Iasi", {{"Vaslui", 92}, {"Neamt", 87}}},
    {"Neamt", {{"Iasi", 87}}}
};

void bfs(const string& startNode, const string& goalNode)
{
    map<string, bool> visited;
    map<string, string> parent; //path  
    map<string, int> cost;
    Queue queue;
    queue.enqueue(startNode);
    visited[startNode] = true;
    cost[startNode] = 0;
    bool goalFound = false;
    string traversal;  // String variable to store the traversal

    while (!queue.isEmptyQ())
    {
        string currentNode = queue.dequeue();
        traversal += currentNode + " ";  // Link the visited node

        if (currentNode == goalNode)
        {
            goalFound = true;
            break;
        }

        for (const auto& neighbor : romaniaMap[currentNode])
        {
            string neighborName = neighbor.first;
            if (!visited[neighborName])
            {
                queue.enqueue(neighborName);
                visited[neighborName] = true;
                parent[neighborName] = currentNode;
                cost[neighborName] = cost[currentNode] + neighbor.second; // Update the cost of the neighbor
            }
        }
    }

    if (goalFound)
    {
        cout << "Goal has been  reached" << endl << endl;
        cout << "Your traversal will be: " << traversal << endl << endl;  // Print the traversal in a single line

        cout << "Path from " << startNode << " to " << goalNode << ": ";

        // Store the path in a vector
        vector<string> path;
        string node = goalNode;
        while (node != startNode)
        {
            path.push_back(node);
            node = parent[node];
        }
        path.push_back(startNode);

        // Print the path in reverse order
        for (int i = path.size() - 1; i >= 0; --i)
        {
            cout << path[i] << " ";

        }
        cout << endl;
        cout << "Cost from " << startNode << " to " << goalNode << ": " << cost[goalNode] << endl;
    }
    else
    {
        cout << "Goal not reachable!" << endl;
    }
}





void ucs(const string& startingNode, const string& destinationNode)
{
    map<string, bool> visited;
    map<string, string> parent;
    map<string, int> cost;
    vector<string> ucs_traversal_output;
    priority_queue<Node, vector<Node>, greater<Node>> pq;

    for (const auto& city : romaniaMap)
    {
        visited[city.first] = false;
        parent[city.first] = "";
        cost[city.first] = numeric_limits<int>::max(); // include <limits>
    }
    pq.push(Node(startingNode, 0));
    cost[startingNode] = 0;

    while (!pq.empty())
    {
        Node current = pq.top();
        pq.pop();
        string currentNode = current.name;

        if (visited[currentNode])
            continue;

        ucs_traversal_output.push_back(currentNode);
        visited[currentNode] = true;

        if (currentNode == destinationNode)
            break;

        for (const auto& neighbor : romaniaMap[currentNode])
        {
            string nextNode = neighbor.first;
            int edgeCost = neighbor.second;

            if (!visited[nextNode])
            {
                int newCost = cost[currentNode] + edgeCost;
                if (newCost < cost[nextNode])
                {
                    cost[nextNode] = newCost;
                    parent[nextNode] = currentNode;
                    pq.push(Node(nextNode, newCost));
                }
            }
        }
    }

    cout << "UCS Traversal: ";
    for (const auto& node : ucs_traversal_output)
    {
        cout << node << " ";
    }
    cout << endl;

    cout << "Path from " << startingNode << " to " << destinationNode << ": ";
    vector<string> path;
    string current = destinationNode;
    while (current != startingNode)
    {
        path.push_back(current);
        current = parent[current];
    }
    path.push_back(startingNode);
    reverse(path.begin(), path.end());
    for (const auto& node : path)
    {
        cout << node << " ";
    }
    cout << endl;
    cout << "Cost from " << startingNode << " to " << destinationNode << ": " << cost[destinationNode] << endl;
}

void dfs(const string& startNode, const string& goalNode)
{
    map<string, bool> visited;
    map<string, string> parent;
    map<string, int> depth;
    map<string, int> cost;
    vector<string> path;
    vector<string> dfs_traversal_output; // Added vector to store traversal

    MyStack stack;
    stack.push(startNode);
    visited[startNode] = true;

    while (!stack.isEmpty())
    {
        string currentNode = stack.arr[stack.top];

        if (currentNode == goalNode)
            break;

        bool foundUnvisitedNeighbor = false;

        for (const auto& neighbor : romaniaMap[currentNode])
        {
            const string& neighborName = neighbor.first;

            if (!visited[neighborName])
            {
                stack.push(neighborName);
                visited[neighborName] = true;
                parent[neighborName] = currentNode;
                depth[neighborName] = depth[currentNode] + 1;
                cost[neighborName] = cost[currentNode] + neighbor.second;
                foundUnvisitedNeighbor = true;
                dfs_traversal_output.push_back(neighborName); // Store traversal
                break;
            }
        }

        if (!foundUnvisitedNeighbor)
        {
            stack.pop();
        }
    }

    // Print the traversal
    cout << "DFS Traversal: " << startNode << " ";
    for (const auto& node : dfs_traversal_output)
    {
        cout << node << " ";
    }
    cout << endl;

    if (visited[goalNode])
    {
        cout << "Path found: ";
        string node = goalNode;
        while (node != startNode)
        {
            path.push_back(node);
            node = parent[node];
        }
        path.push_back(startNode);
        reverse(path.begin(), path.end());

        for (const string& city : path)
        {
            cout << city;
            if (city != goalNode)
                cout << " -> ";
        }
        cout << endl;

        int totalCost = cost[goalNode];
        cout << "Total Cost from " << startNode << " to " << goalNode << " is " << totalCost << endl;
    }
    else
    {
        cout << "Path not found." << endl;
    }
}




bool dlsUtil(const string& currentNode, const string& destinationNode, map<string, bool>& visited,
    map<string, string>& parent, vector<string>& dls_traversal_output, int depth, int maxDepth)
{
    visited[currentNode] = true;
    dls_traversal_output.push_back(currentNode);

    if (currentNode == destinationNode)
        return true;

    if (depth >= maxDepth)
        return false;

    for (const auto& neighbor : romaniaMap[currentNode])
    {
        string nextNode = neighbor.first;

        if (!visited[nextNode])
        {
            parent[nextNode] = currentNode;
            if (dlsUtil(nextNode, destinationNode, visited, parent, dls_traversal_output, depth + 1, maxDepth))
                return true;
        }
    }

    return false;
}

void dls(const string& startingNode, const string& destinationNode, int maxDepth)
{
    map<string, bool> visited;
    map<string, string> parent;
    vector<string> dls_traversal_output;

    for (const auto& city : romaniaMap)
    {
        visited[city.first] = false;
        parent[city.first] = "";
    }

    bool found = dlsUtil(startingNode, destinationNode, visited, parent, dls_traversal_output, 0, maxDepth);

    cout << "DLS Traversal: ";
    for (const auto& node : dls_traversal_output)
    {
        cout << node << " ";
    }
    cout << endl;

    if (found)
    {
        cout << "Path from " << startingNode << " to " << destinationNode << ": ";
        vector<string> path;
        string current = destinationNode;
        while (current != startingNode)
        {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(startingNode);
        reverse(path.begin(), path.end());
        for (const auto& node : path)
        {
            cout << node << " ";
        }
        cout << endl;
    }
    else
    {
        cout << "Path from " << startingNode << " to " << destinationNode << " not found within the specified depth." << endl;
    }
}

void iddfs(const string& startingNode, const string& destinationNode)
{
    int maxDepth = 0;

    while (true)
    {
        map<string, bool> visited;
        map<string, string> parent;
        vector<string> iddfs_traversal_output;

        for (const auto& city : romaniaMap)
        {
            visited[city.first] = false;
            parent[city.first] = "";
        }

        bool found = dlsUtil(startingNode, destinationNode, visited, parent, iddfs_traversal_output, 0, maxDepth);

        cout << "IDDFS Traversal (Depth " << maxDepth << "): ";
        for (const auto& node : iddfs_traversal_output)
        {
            cout << node << " ";
        }
        cout << endl;

        if (found)
        {
            cout << "Path from " << startingNode << " to " << destinationNode << ": ";
            vector<string> path;
            string current = destinationNode;
            while (current != startingNode)
            {
                path.push_back(current);
                current = parent[current];
            }
            path.push_back(startingNode);
            reverse(path.begin(), path.end());
            for (const auto& node : path)
            {
                cout << node << " ";
            }
            cout << endl;
            break;
        }

        maxDepth++;
    }
}

int main()
{
    string startingNode;
    string destinationNode;
    int algorithmChoice;

    while (1)
    {
        cout << "You have 5 algoritms to search in ROMANIA MAP : " << endl << endl;
        cout << "Select an algorithm from (1-5) to start search : " << endl << endl;
        cout << "#1 <-------------        Breadth-First Search (BFS)        --------------> " << endl << endl;
        cout << "#2 <-------------        Uniform Cost Search (UCS)         -------------->" << endl << endl;
        cout << "#3 <-------------        Depth-First Search (DFS)          -------------->" << endl << endl;
        cout << "#4 <-------------        Depth-Limited Search(DLS)         -------------->" << endl << endl;
        cout << "#5 <-------------    Iterative deepening search(IDDFS)     -------------->" << endl << endl;
        cout << "#6 <-------------                  Exit                    -------------->" << endl << endl;

        cout << "Enter Your choice please from the previous algorthims : ";
        cin >> algorithmChoice;
        if ((algorithmChoice < 6) && (algorithmChoice > 0))
        {
            cout << "Enter start point [First Alphabet Must Be CAPITAL]: ";
            cin >> startingNode;
            cout << "Enter Your Goal you need to reach to [First Alphabet Must Be CAPITAL]: ";
            cin >> destinationNode;
        }
        else
        {
            return 0;
        }
        switch (algorithmChoice)
        {
        case 1:
            bfs(startingNode, destinationNode);
            break;
        case 2:
            ucs(startingNode, destinationNode);
            break;
        case 3:
            dfs(startingNode, destinationNode);
            break;
        case 4:
            int maxDepth;
            cout << "Enter the MaxDepth :";
            cin >> maxDepth;
            dls(startingNode, destinationNode, maxDepth);
            break;
        case 5:
            iddfs(startingNode, destinationNode);
            break;
        case 6:
            exit(1);
        }
        cout << "---------------------------------------------------------------------------------------------------------------------\n";

    }
    return 0;
}
