/* By Tyler Clarke. Any other work, by any person under any circumstances, will be duly accredited.
    A threaded HTTP server in C++, primarily for the use of my robotics team. Works on any platform supporting the Unix TCP socket api.
*/

#ifndef HTTPSERVER_HPP
#define HTTPSERVER_HPP
// Socket includes
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstring>
#include <fcntl.h>
#include <sys/poll.h>

// STD includes
#include <thread>
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <mutex>
#include <functional>

// Otherwise
#include <signal.h>

// Only for testing
#include <unistd.h>
#include <iostream>

// Processing positions for requests.
#define DETERMINING_HTTP_METHOD 0
#define DETERMINING_HTTP_PATH 1
#define DETERMINING_HTTP_VERSION 2
#define READING_HTTP_HEADERS_P1 3 // Reading the name
#define READING_HTTP_HEADERS_P2 4 // Reading the value
#define READ_CONTENT 5

// HTTP methods
#define HTTP_METHOD_GET 0
#define HTTP_METHOD_POST 1
#define HTTP_METHOD_DELETE 2
#define HTTP_METHOD_PUT 3
#define HTTP_METHOD_PATCH 4
#define HTTP_METHOD_OPTIONS 5
#define HTTP_METHOD_HEAD 6
#define HTTP_METHOD_INVALID 255

// HTTP versions, for now only these are supported.
#define HTTP_VERSION_1_1 0
#define HTTP_VERSION_INVALID 255

#define HTTPTEMPLATEARGS HTTPRequest*, HTTPResponse*, Client*
#define HTTPPASSARGS req, ron, client
#define HTTPARGS HTTPRequest* req, HTTPResponse* ron, Client* client

// These allow a main spec to be built.

// Note that error handling should be done by the code running the server! This program will assign HTTP_*_INVALID to all fields that permit it,
// if it catches a problem.

struct HTTPHeader{
    std::string name; // Std::strings are fairly light and efficient and are easier to do many operations on.
    std::string value; // We'll use them until somebody complains, then we'll ignore the complainers so viciously they won't know what didn't notice them.
};

struct HTTPRequest{
    uint8_t procpos = DETERMINING_HTTP_METHOD; // Processing position.

    uint8_t method;      // These are all set during processing.
    uint8_t HTTPVersion;
    std::string url;
    std::string content;
    unsigned int contentLength = 0; // The header.

    std::vector<HTTPHeader*> headers;

    std::map<std::string, std::string> cookies;

    HTTPHeader *_currentHeader; // Header buffer.
    std::string _currentData; // This is a multi-purpose buffer.
    bool _ignoringSpaces = true; // This is for when it ignores whitespace between the name and value of a header.

    HTTPHeader* getHeader(std::string name){
        for (unsigned int i; i < headers.size(); i ++){
            if (headers[i] -> name == name){
                return headers[i];
            }
        }
        return nullptr;
    }

    void dealloc(){
        free(_currentHeader);
        for (unsigned int x = 0; x < headers.size(); x++){
            free(headers[x]);
        }
    }
};

struct HTTPResponse{
    HTTPRequest *origReq;
    std::vector<HTTPHeader*> headers;
    std::string content;
    uint16_t status;

    // Automatically available headers:
    bool keepAlive = true;
    std::string contentType = "text/plain";

    std::map<std::string, std::string> cookies;

    void dealloc(){
        //free(origReq);
        for (unsigned int x = 0; x < headers.size(); x++){
            std::cout << "Freeing header " << headers[x] -> name << std::endl;
            delete headers[x];
        }
    }
};

// Client, is it really that hard to understand? Badadadummmmmm.
struct Client{
    HTTPRequest *currentRequest;
    bool hasRequest = false;
    int sock;
    void *state = nullptr;
};

// HTTP server structure, I always use structs because classes are eww. It has all the necessary functions to do things.
struct HTTPServer{
    int serverSock = -1; // We don't want this to accidentally write to stdin/out if it is not assigned.
    // Yet.
    struct sockaddr_in address;
    socklen_t addrsize = sizeof(address);

    std::vector<Client*> clients;

    std::function<void(HTTPTEMPLATEARGS)> requestCallback;
    std::function<void(Client*)> disconnectedCallback;

    std::mutex readMutex;

    pollfd* pollFdList;

    void recalcPollFdList(){
        if (clients.size() > 0){
            pollFdList = (pollfd*)realloc(pollFdList, sizeof(pollfd) * clients.size());
            for (unsigned int i = 0; i < clients.size(); i ++){
                pollFdList[i].fd = clients[i] -> sock;
                pollFdList[i].events = POLLIN;
            }
        }
    }

    static void accepter(HTTPServer* self){ // The self thing is a workaround; because thread functions can't be members, you have to pass in `this` the hard way.
        while (true){
            int cliSock = accept(self -> serverSock, (struct sockaddr*)&self -> address, &self -> addrsize);
            self -> readMutex.lock();
            Client *client = new Client;
            client -> sock = cliSock;
            fcntl(cliSock, F_SETFL, O_NONBLOCK);
            self -> clients.push_back(client);
            self -> recalcPollFdList();
            self -> readMutex.unlock();
            printf("Got new client!\n");
        }
    }

    void sendResponse(HTTPResponse* response, Client *client){
        if (response -> origReq -> HTTPVersion == HTTP_VERSION_1_1){
            send(client -> sock, "HTTP/1.1 ", 9, 0);
        }
        else {
            send(client -> sock, "Invalid protocol version. Please go away.", 41, 0);
            return;
        }
        char code[5];
        snprintf(code, 5, "%d\r\n", response -> status);
        send(client -> sock, code, 5, 0); // 3 for response code + 2 for CRLF
        for (unsigned int x = 0; x < response -> headers.size(); x ++){
            HTTPHeader *curHeader = response -> headers[x];
            send(client -> sock, curHeader -> name.c_str(), curHeader -> name.size(), 0);
            send(client -> sock, ": ", 2, 0);
            send(client -> sock, curHeader -> value.c_str(), curHeader -> value.size(), 0);
            send(client -> sock, "\r\n", 2, 0);
        }
        // Autogen headers here:
        // Connection (for keep-alive and stuff):
        send(client -> sock, "Connection: ", 12, 0);
        if (response -> keepAlive){
            send(client -> sock, "keep-alive\r\n", 12, 0);
        }
        else{
            send(client -> sock, "close\r\n", 7, 0);
        }
        // Cookies
        for (const auto& pair : response -> cookies){
            std::cout << "Set-Cookie: " + pair.first + "=" + pair.second;
            //send(client -> sock, "Set-Cookie: " + pair.first
        }
        // Content-Length:
        std::string contentLengthHeader = "Content-Length: ";
        contentLengthHeader += std::to_string(response -> content.size());
        contentLengthHeader += "\r\n";
        send(client -> sock, contentLengthHeader.c_str(), contentLengthHeader.size(), 0);
        // Content-Type:
        send(client -> sock, "Content-Type: ", 14, 0);
        send(client -> sock, response -> contentType.c_str(), response -> contentType.size(), 0);
        send(client -> sock, "\r\n", 2, 0);

        // Send the actual content:
        send(client -> sock, "\r\n", 2, 0);
        send(client -> sock, response -> content.c_str(), response -> content.size(), 0);
    }

    void closeAll(){
        readMutex.lock();
        for (unsigned int x = 0; x < clients.size(); x ++){
            close(clients[x] -> sock);
            clients.erase(clients.begin() + x);
        }
        close(serverSock);
        std::cout << "Freed, and done." << std::endl;
        // No need to unlock the mutex, the scope is being destroyed.
    }

    static void reader(HTTPServer* self){
        while (true){
            self -> readMutex.lock();
            if (self -> clients.size() > 0){ // Initial connection shouldn't take time.
                poll(self -> pollFdList, self -> clients.size(), 750); // 0.75 second timeout, this gives it a 0.5 second idle time every iteration (assuming no new clients)
            }
            unsigned int curSize = self -> clients.size();
            for (unsigned int x = 0; x < curSize; x ++){
                if (!self -> pollFdList[x].revents & POLLIN){
                    continue;
                }
                Client *client = self -> clients[x];
                char buffer;
                if (recv(client -> sock, &buffer, 1, 0) == 0){ // Disconnect dead sockets.
                    self -> disconnectedCallback(client);
                    close(client -> sock);
                    free(client);
                    self -> clients.erase(self -> clients.begin() + x);
                    printf("Socket disconnected.\n");
                    self -> recalcPollFdList();
                    break; // Dirty, but effective.
                }
                if (!client -> hasRequest){
                    client -> currentRequest = new HTTPRequest;
                    client -> hasRequest = true; // Not including this was causing me annoying buggies and segfaultification, I think.
                }
                HTTPRequest *req = client -> currentRequest;
                int length = 0;
                if (req -> procpos == DETERMINING_HTTP_METHOD){
                    for (uint8_t i = 0; i < 7; i ++){
                        length = recv(client -> sock, &buffer, 1, 0); // Buffer, length, flags.
                        if (length != 1){ // Any non-one case
                            break;
                        }
                        if (buffer == ' '){ // It has reached the end if the buffer is space.
                            req -> procpos = DETERMINING_HTTP_PATH;
                            if (req -> _currentData == "GET"){
                                req -> method = HTTP_METHOD_GET;
                            }
                            else if (req -> _currentData == "POST"){
                                req -> method = HTTP_METHOD_POST;
                            }
                            else if (req -> _currentData == "DELETE"){
                                req -> method = HTTP_METHOD_DELETE;
                            }
                            else if (req -> _currentData == "PUT"){
                                req -> method = HTTP_METHOD_PUT;
                            }
                            else if (req -> _currentData == "PATCH"){
                                req -> method = HTTP_METHOD_PATCH;
                            }
                            else if (req -> _currentData == "HEAD"){
                                req -> method = HTTP_METHOD_HEAD;
                            }
                            else if (req -> _currentData == "OPTIONS"){
                                req -> method = HTTP_METHOD_OPTIONS;
                            }
                            else {
                                req -> method = HTTP_METHOD_INVALID;
                            }
                            req -> _currentData = ""; // Empty it
                            break;
                        }
                        req -> _currentData += buffer;
                    }
                }
                if (req -> procpos == DETERMINING_HTTP_PATH){
                    while (true){ // T'aint true, it uses break statements.
                        length = recv(client -> sock, &buffer, 1, 0); // Buffer, length, flags.
                        if (buffer == ' '){
                            req -> url = req -> _currentData;
                            req -> _currentData = ""; // Clear it
                            req -> procpos = DETERMINING_HTTP_VERSION; // It can support any HTTP version, this is just to get past it.
                            break;
                        }
                        if (length != 1){
                            break; // Nothing to do but break and wait.
                        }
                        req -> _currentData += buffer;
                    }
                }
                if (req -> procpos == DETERMINING_HTTP_VERSION){
                    for (uint8_t i = 0; i < 8; i ++){
                        length = recv(client -> sock, &buffer, 1, 0); // Buffer, length, flags.
                        if (buffer == '\n'){
                            req -> procpos = READING_HTTP_HEADERS_P1;
                            if (req -> _currentData == "HTTP/1.1"){
                                req -> HTTPVersion = HTTP_VERSION_1_1;
                            }
                            else {
                                req -> HTTPVersion = HTTP_VERSION_INVALID;
                            }
                            req -> _currentData = "";
                            break;
                        }
                        if (length != 1){
                            break;
                        }
                        if (buffer != ' ' && buffer != '\r' && buffer != '\n'){ // Both of these characters will throw it off.
                            req -> _currentData += buffer;
                        }
                    }
                    // Length of an HTTP version is 8 (HTTP/1.1 is the only one we intend to support as of this writing)
                }
                if (req -> procpos == READING_HTTP_HEADERS_P1){
                    req -> _ignoringSpaces = true; // Gotta reset it.
                    req -> _currentData += buffer; // Because otherwise we lose this byte. Idk why.
                    while (true){ // See the other one that does this, the comment is the same.
                        length = recv(client -> sock, &buffer, 1, 0); // Buffer, length, flags.
                        if (length != 1){
                            break;
                        }
                        if (buffer == '\n'){ // Never happens until the empty line before body.
                            req -> procpos = READ_CONTENT;
                            break;
                        }
                        if (buffer == ':'){
                            req -> procpos = READING_HTTP_HEADERS_P2;
                            req -> _currentHeader = new HTTPHeader;
                            req -> _currentHeader -> name = req -> _currentData;
                            req -> _currentData = "";
                            break;
                        }
                        req -> _currentData += buffer;
                    }
                }
                if (req -> procpos == READING_HTTP_HEADERS_P2){
                    while (true){
                        length = recv(client -> sock, &buffer, 1, 0); // Buffer, length, flags.
                        if (length != 1){
                            break;
                        }
                        if (buffer == '\n'){
                            req -> _currentHeader -> value = req -> _currentData;
                            req -> _currentData = "";
                            req -> procpos = READING_HTTP_HEADERS_P1;
                            if (req -> _currentHeader -> name == "Content-Length"){
                                req -> contentLength = std::stoi(req -> _currentHeader -> value);
                            }
                            else if (req -> _currentHeader -> name == "Cookie"){
                                std::string cName;
                                std::string cVal;
                                uint8_t cPhase = 0;
                                for (unsigned int x = 0; x < req -> _currentHeader -> value.size(); x ++){
                                    char tbu = req -> _currentHeader -> value.at(x);
                                    if (cPhase == 0){
                                        if (tbu == '='){
                                            cPhase = 1;
                                        }
                                        else{
                                            cName += tbu;
                                        }
                                    }
                                    else if (cPhase == 1){
                                        if (tbu == ';'){
                                            req -> cookies[cName] = cVal;
                                            cName = "";
                                            cVal = "";
                                            cPhase = 0;
                                        }
                                        else{
                                            cVal += tbu;
                                        }
                                    }
                                }
                                if (cName != ""){
                                    req -> cookies[cName] = cVal;
                                }
                            }
                            else{
                                std::cout << "Header name:" << req -> _currentHeader -> name << "!" << std::endl;
                                req -> _currentHeader = nullptr;
                                req -> headers.push_back(req -> _currentHeader);
                            }
                            break;
                        }
                        if (req -> _ignoringSpaces && buffer != ' '){
                            req -> _ignoringSpaces = false;
                        }
                        if (!(buffer == '\r') && !req -> _ignoringSpaces){
                            req -> _currentData += buffer;
                        }
                    }
                }
                if (req -> procpos == READ_CONTENT){
                    char* buf = (char*)malloc(req -> contentLength + 1);
                    buf[req -> contentLength] = 0; // Add a 0 at the end, so it is a valid string.
                    length = recv(client -> sock, buf, req -> contentLength, 0);
                    req -> content += buf;
                    free(buf);
                    if (req -> content.length() >= req -> contentLength){ // That's all, folks!
                        req -> procpos = DETERMINING_HTTP_METHOD;
                        HTTPResponse *ron = new HTTPResponse;
                        ron -> origReq = req;
                        self -> requestCallback(HTTPPASSARGS);
                        self -> sendResponse(ron, client);
                        req -> dealloc();
                        free(req);
                        ron -> dealloc();
                        free(ron);
                    }
                }
            }
            self -> readMutex.unlock();
        }
    }

    HTTPServer(unsigned int port, std::function<void(HTTPTEMPLATEARGS)> rCbck, std::function<void(Client*)> dCbck, unsigned int timeout = 2){
        signal(SIGPIPE, SIG_IGN);
        requestCallback = rCbck;
        disconnectedCallback = dCbck;
        serverSock = socket(AF_INET, SOCK_STREAM, 0);
        memset(&address, 0, sizeof(struct sockaddr_in));
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = INADDR_ANY;

        long reuse = 1;
        if (setsockopt(serverSock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(long)) == -1){
            std::cout << "Can't set socket options. Time to die." << std::endl;
            printf(strerror(errno));
            exit(1);
        }
        while (bind(serverSock, (struct sockaddr*)&address, sizeof(struct sockaddr_in)) == -1){
            printf("Bind failed. Trying again in %d seconds.\n", timeout);
            usleep(timeout * 1000000);
        }
        printf("Successfully bound to port %d\n", port);

        listen(serverSock, 25); // Listen for maximum 25 peoples

        std::thread accepterThread(accepter, this);
        accepterThread.detach();
        std::thread readerThread(reader, this);
        readerThread.detach();
        recalcPollFdList();
    }

    void run(){

    }
};
#endif
