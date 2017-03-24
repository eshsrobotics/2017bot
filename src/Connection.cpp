#include "Connection.h"

#include <mutex>
#include <thread>
#include <sstream>
#include <condition_variable>

#include <unistd.h>     // write()
#include <string.h>     // strerror()
#include <netdb.h>      // struct addrinfo
#include <sys/socket.h>

#include <time.h> // For the reentrant and POSIX-standard localtime_r()


using std::move;
using std::mutex;
using std::thread;
using std::string;
using std::vector;
using std::cv_status;
using std::defer_lock;
using std::lock_guard;
using std::unique_lock;
using std::stringstream;
using std::runtime_error;
using std::condition_variable;

using namespace std::chrono;


namespace robot {

// Create a connection that's not connected to anything.
Connection::Connection(TransmissionBuffer& buffer)
    : connecting_(false), buffer_(buffer), fd(-1), connectedAddress_(),
      connectedPort_(-1), parameters() {

    if (++connection_count == 1) {

        // Tell the runtime to ignore SIGPIPE.
        struct sigaction new_sigaction;
        new_sigaction.sa_handler = SIG_IGN;
        sigemptyset(&new_sigaction.sa_mask);
        new_sigaction.sa_flags = 0;
        int result = sigaction(SIGPIPE, &new_sigaction, &original_sigaction);

        if (result < 0) {

            stringstream stream;
            stream << "*** PELIGRO! *** Unable to register a sigaction handler because sigaction() returned "
                   << result
                   << ".  Any remote disconnection will now terminate the program!";

            buffer_.logMessage(TransmissionBuffer::debug, stream.str());

        } else {

            buffer_.logMessage(TransmissionBuffer::debug, "Connection::Connection: sigaction handler for SIGPIPE installed successfully.");

        }
    }
}


// Move construction: *this takes control of other's assets, leaving other as
// an empty husk.
Connection::Connection(Connection&& other)
    : connecting_(other.connecting_), buffer_(other.buffer_), fd(other.fd),
      connectedAddress_(move(other.connectedAddress_)),
      connectedPort_(other.connectedPort_),
      parameters(move(other.parameters)) {

    other.connecting_ = false;
    other.fd = -1;
    other.connectedAddress_ = "";
    other.connectedPort_ = -1;
}


// Move assignment: *this takes control of other's assets, leaving other as an
// empty husk.
Connection& Connection::operator= (Connection&& other) {
    connecting_ = other.connecting_;
    buffer_ = move(other.buffer_);
    fd = other.fd;
    connectedAddress_ = move(other.connectedAddress_);
    connectedPort_ = other.connectedPort_;
    parameters = move(other.parameters);

    other.connecting_ = false;
    other.fd = -1;
    other.connectedAddress_ = "";
    other.connectedPort_ = -1;

    return *this;
}


// The destructor disconnects and unregisters the signal handler.
Connection::~Connection() {
    disconnect();

    if (--connection_count == 0) {
        int result = sigaction(SIGPIPE, &original_sigaction, nullptr);

        if (result < 0) {

            stringstream stream;
            stream << "Connection::~Connection: Warning: sigaction() returned " << result
                   << " while restoring the old SIGPIPE handler.";

            buffer_.logMessage(TransmissionBuffer::debug, stream.str());

        } else {

            buffer_.logMessage(TransmissionBuffer::debug, "Connection::~Connection: original sigaction handler for SIGPIPE restored successfully.");

        }
    }
}


// Writes a string to the underlying socket if the connection is active.
bool Connection::write(const string& s, TransmissionBuffer::LogType logTypeForErrors) {

    if (fd >= 0) {
        ssize_t result = ::write(fd, s.c_str(), s.size());

        if (static_cast<size_t>(result) == s.size()) {

            return true;

        } else if (result >= 0) {

            stringstream stream;
            stream << "write: Warning: Only wrote " << result << " out of "
                   << s.size()
                   << "bytes from last message to socket.  Perhaps there was a network interruption?";
            buffer_.logMessage(logTypeForErrors, stream.str());

        } else {

            int old_errno = errno; // Any subsequent glibc call might change it.

            if (old_errno == EPIPE) {

                // This is what we get when there's a remote disconnection while
                // ignoring the SIGPIPE signal.  We handle it the same way we
                // would handle SIGPIPE: by explicitly disconnecting on our end

                stringstream stream;
                stream << "write: Warning: Received received error code " << EPIPE
                       << " (EPIPE) while trying to write to socket file descriptor "
                       << fd << ".  Connection to " << connectedAddress_ << ":"
                       << connectedPort_ << " must have closed unexpectedly.";

                buffer_.logMessage(logTypeForErrors, stream.str());
                disconnect();

            } else {

                char* message = strerror(old_errno);
                stringstream stream;
                stream << "write: ERROR: Can't write to socket for file descriptor "
                       << fd << ": \"" << message << "\" (errno = "
                       << old_errno << ")";
                buffer_.logMessage(logTypeForErrors, stream.str());
            }
        }
    }
    return false;
}


/// This low-level routine opens a socket to the given address and port and
/// then returns its descriptor.  This is the business part of the thread
/// function run by each of the connection threads in createClientSocket().
///
/// We employ the modern getaddrinfo() approach here, which is much more
/// concise than getprotobyname() et al.
///
/// @param addressToTry A DNS name or IPv4 address string.
/// @param port A port number to connect to on the addressToTry.
/// @param buffer The TransmissionBuffer to use for queuing error messages if
///               we end up having a connection problem.
/// @param logTypeForErrors How we will classify any connection error messages
///                         at the time we log them.
/// @return A file descriptor representing the valid, connected socket, or -1
///         upon failure.
int Connection::_createClientSocket(const string& addressToTry, int port, TransmissionBuffer& buffer, TransmissionBuffer::LogType logTypeForErrors) {

    stringstream stream;
    stream << port;
    string portString = stream.str();

    stream.str("");
    stream << "_createClientSocket [" << std::this_thread::get_id() << "]";
    string name = stream.str();

    stream.str("");
    stream << name << ": Trying to connect to " << addressToTry << ":" << port;
    buffer.logMessage(TransmissionBuffer::debug, stream.str());

    addrinfo hints;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = 0;
    hints.ai_flags = 0;
    addrinfo *result;
    int errorCode = getaddrinfo(addressToTry.c_str(), portString.c_str(), &hints, &result);

    if (errorCode == 0) {

        // DNS resolution worked.  Time to connect.
        int fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
        errorCode = ::connect(fd, result->ai_addr, result->ai_addrlen);

        // We don't need the addrinfo data structure that
        // getaddrinfo() allocated anymore, regardless of whether we
        // connected successfully or not.
        freeaddrinfo(result);

        if (errorCode == 0) {

            stream.str("");
            stream << name << ": Successfully connected to " << addressToTry << ":"
                   << port << " with file descriptor " << fd << ".";
            buffer.logMessage(TransmissionBuffer::debug, stream.str());
            return fd;

        } else {

            // If we made it here, we did the DNS resolution, but we couldn't
            // connect to that address.
            int old_errno = errno;    // Any subsequent glibc call might change it.
            char* message = strerror(old_errno);

            stream.str("");
            stream << name << ": WARNING: Can't connect to "
                   << addressToTry << ":" << port << ": \"" << message
                   << "\" (errno = " << old_errno << ")";
            buffer.logMessage(logTypeForErrors, stream.str());
            return -1;
        }

    }

    // If we made it here, DNS resolution failed.
    stream.str("");
    stream << name << ": WARNING: Can't locate host named '"
           << addressToTry << "' on the network: " << gai_strerror(errorCode);
    buffer.logMessage(logTypeForErrors, stream.str());
    return -1;
}


// ==========================================================================
// Attempts to connect to one of the given addresses or DNS names using the
// given port.  Gives up if no connection can be made after
// timeoutInMilliseconds seconds have elapsed.

void Connection::connect(const std::vector<std::string>& addressesToTry,
                         int port,
                         int timeoutInMilliseconds,
                         TransmissionBuffer::LogType logTypeForErrors) {

    // Allows the main thread (i.e., us, right here) to be asynchronously
    // notified whenever one of our child threads is able to successfully
    // connect.  But the _real_ advantage of using a condition variable is
    // that condition_variable::wait_for() allows us to leave early if the
    // timeout has expired.
    //
    // Connections to an address that can't be found on the network take a
    // long time to fail, and we don't want that to block the robot's camera
    // subsystem.
    condition_variable cv;

    // The condition variable uses file_descriptor_lock to create critical
    // sections where needed.  The child connection threads also rely on the
    // underlying mutex to safely modify the shared member data under *this.
    mutex file_descriptor_mutex;
    unique_lock<mutex> file_descriptor_lock(file_descriptor_mutex, defer_lock);

    // This flag is used so that all of the other child threads that have not
    // managed to connect will leave *this alone.
    bool connection_made = false;

    // This is our first official (or most recent) connection attempt; set
    // these variables to let reconnect() know that it will be okay to
    // reconnect should we fail.
    parameters.addressesToTry = addressesToTry;
    parameters.port = port;
    parameters.logTypeForErrors = logTypeForErrors;
    parameters.timeoutInMilliseconds = timeoutInMilliseconds;

    // The function that all child threads run.
    //
    // It attempts to connect to a single address and port.
    auto connectionThreadFunction =
        [this, &cv, &file_descriptor_mutex, &file_descriptor_lock, &connection_made] (const string& addressToTry, int port, TransmissionBuffer::LogType logTypeForErrors) {

        // Perform the potentially-expensive connection, which will block
        // this thread until it completes.
        int my_fd = _createClientSocket(addressToTry, port, this->buffer_, logTypeForErrors);

        if (my_fd > 0) { // We connected!

            if (!connection_made) { // We connected first.

                // Write the shared data safely.
                {
                    lock_guard<mutex> lock(file_descriptor_mutex);
                    this->fd = my_fd;
                    this->connectedAddress_ = addressToTry;
                    this->connectedPort_ = port;
                }

                // Let our calling thread know we're ready.  (It might have
                // already returned if we took too long, though.)
                cv.notify_one();

            } else { // Some other thread beat us to the punch.

                stringstream stream;
                stream << "createClientSocket [" << std::this_thread::get_id()
                       << "]: Successfully connected, but another thread has already connected.  Closing this thread's file descriptor ("
                       << fd << ").";
                lock_guard<mutex> lock(file_descriptor_mutex);
                this->buffer_.logMessage(TransmissionBuffer::debug, stream.str());

                // Our fd is now useless!
                close(my_fd);
            }
        } else {

            // Let's hope some other thread succeeds where we failed.

            // stringstream stream;
            // stream << "createClientSocket [" << std::this_thread::get_id()
            //        << "]: WARNING: Could not connect to " << addressToTry << ":" << port << ".";
            // buffer.logMessage(TransmissionBuffer::debug, stream.str());
        }
    };

    // Spawn multiple parallel threads to connect to all of the addressesToTry
    // at once.
    //
    // There is a _slight_ danger here that one of the threads will notify
    // before we have had a chance to finish spawning the other threads and
    // wait on the condition variable.  If that's a problem, having each
    // thread wait for a handful of milliseconds at the start should help.
    connecting_ = true;
    vector<thread> connectionThreads;
    for (string addressToTry : addressesToTry) {
        connectionThreads.push_back(thread(connectionThreadFunction, addressToTry, port, logTypeForErrors));

        // A thread in C++ that exits without being joined or detached
        // terminate()s, so we need to sever our ties with the connection
        // threads right away.
        if (connectionThreads.back().joinable()) {
            connectionThreads.back().detach();
        }
    }

    // Wait for a thread to notify us, but our time is limited.
    milliseconds timeout(timeoutInMilliseconds);
    if (cv.wait_for(file_descriptor_lock, timeout) == cv_status::no_timeout) {

        connection_made = true;

        // The other detached threads will get the hint eventually, but in the
        // meantime, we have to go.

        stringstream stream;
        stream << "createClientSocket [main]: Returning file descriptor " << fd << ".";
        buffer_.logMessage(TransmissionBuffer::debug, stream.str());
        connecting_ = false;
        return;
    }

    // If control made it here, we obviously timed out.
    connecting_ = false;
    stringstream stream;
    stream << "createClientSocket [main]: ERROR: No connections succeeded within "
           << timeout.count() << " milliseconds.  Giving up.\n";
    throw runtime_error(stream.str());
}


// Try to connect to one of the given addresses using the given port.
void Connection::disconnect() {
    if (connecting_ == true) {
        // We haven't connected yet, and we're still trying.  There's nothing
        // to disconnect, and the threads in connect() would overwrite
        // whatever we did here anyway.
        return;
    }

    if (fd >= 0) {
        close(fd);
        stringstream stream;
        stream << "disconnect: Closed file descriptor " << fd;
        buffer_.logMessage(TransmissionBuffer::debug, stream.str());

        // Let the outside world know we have no connection.
        fd = -1;
        connectedAddress_ = "";
        connectedPort_ = -1;
    }
}


// ==========================================================================
// Reconnect to whatever we successfully connected to last time.

void Connection::reconnect() {

    if (parameters.addressesToTry.size() == 0) {
        // We can't *re*-connect until we've connected at least once.
        return;
    }

    if (connecting_ == true) {
        // A connection is currently in progress.  Let's not interrupt it.
        return;
    }

    // If control makes it here, we connected successfully at least once in
    // the past, and are not making a connection now.

    if (connected()) {
        disconnect();
    }
    connect(parameters.addressesToTry, parameters.port, parameters.timeoutInMilliseconds, parameters.logTypeForErrors);
}


// ==========================================================================
// One-liner methods.

int Connection::descriptor() const { return fd; }
string Connection::address() const { return connectedAddress_; }
int Connection::port() const { return connectedPort_; }
bool Connection::connected() const { return (fd > 0); }


} // end (namespace robot)
