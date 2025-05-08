#include <cpp_epuck/NetworkFactory.hpp>
#include <gmock/gmock.h>
#include <utility>

// Mock implementations for testing
class MockIoContext : public IIoContext
{
public:
    MOCK_METHOD(void, run, (), (override));

    MOCK_METHOD(void, stop, (), (override));

    MOCK_METHOD(asio::io_context &, get_native_context, (), (override));

    // This is needed for tests that must interact with the real io_context
    asio::io_context real_context;

    asio::io_context &get_native_context_impl() { return real_context; }
};

class MockTcpEndpoint : public ITCPEndpoint
{
public:
    MOCK_METHOD(asio::ip::tcp::endpoint &, get_native_endpoint, (), (override));

    MOCK_METHOD(asio::ip::address, address, (), (override));

    MOCK_METHOD(uint16_t, port, (), (override));
};

class MockTcpSocket : public ITcpSocket
{
public:
    MOCK_METHOD(void, connect, (const asio::ip::tcp::endpoint &endpoint), (override));

    MOCK_METHOD(size_t, send, (const asio::const_buffer &buffer), (override));

    MOCK_METHOD(size_t, receive, (const asio::mutable_buffer &buffer), (override));

    MOCK_METHOD(void, close, (), (override));

    MOCK_METHOD(bool, is_open, (), (override));

    MOCK_METHOD(asio::ip::tcp::socket &, get_native_socket, (), (override));

private:
    asio::mutable_buffer _receive_buffer;
};

class MockUdpEndpoint : public IUDPEndpoint
{
public:
    MOCK_METHOD(asio::ip::udp::endpoint &, get_native_endpoint, (), (override));

    MOCK_METHOD(asio::ip::address, address, (), (override));

    MOCK_METHOD(uint16_t, port, (), (override));
};

class MockUdpSocket : public IUdpSocket
{
public:
    MOCK_METHOD(void, open, (), (override));

    MOCK_METHOD(void, bind, (IUDPEndpoint & endpoint), (override));

    MOCK_METHOD(size_t, send_to, (const asio::const_buffer &buffer, IUDPEndpoint &destination), (override));

    MOCK_METHOD(size_t, receive_from, (const asio::mutable_buffer &buffer, IUDPEndpoint &sender), (override));

    MOCK_METHOD(void, close, (), (override));

    MOCK_METHOD(bool, is_open, (), (override));

    MOCK_METHOD(asio::ip::udp::socket &, get_native_socket, (), (override));

private:
    asio::mutable_buffer _receive_buffer;
};

class MockResolver : public IResolver
{
public:
    MOCK_METHOD(asio::ip::tcp::resolver::results_type, resolve_tcp,
                (const std::string &host, const std::string &service), (override));

    MOCK_METHOD(asio::ip::udp::resolver::results_type, resolve_udp,
                (const std::string &host, const std::string &service), (override));
};

class MockTimer : public ITimer
{
public:
    MOCK_METHOD(void, expires_after, (const asio::steady_timer::duration &expiry_time), (override));

    MOCK_METHOD(void, expires_at, (const asio::steady_timer::time_point &expiry_time), (override));

    MOCK_METHOD(void, wait, (), (override));

    MOCK_METHOD(void, cancel, (), (override));
};

// Mock factory for testing
class MockNetworkFactory : public INetworkFactory
{
public:
    // These methods will return mock objects that can be configured in tests
    std::shared_ptr<IIoContext> create_io_context() override
    {
        auto mock = std::make_shared<MockIoContext>();
        // Set up default behavior
        ON_CALL(*mock, get_native_context())
            .WillByDefault(testing::Invoke(mock.get(), &MockIoContext::get_native_context_impl));
        return mock;
    }

    std::shared_ptr<ITCPEndpoint> create_tcp_endpoint() override { return std::make_shared<MockTcpEndpoint>(); }

    std::shared_ptr<ITCPEndpoint> create_tcp_endpoint(asio::ip::address_v4 /*addr*/, uint16_t /*port*/) override
    {
        return std::make_shared<MockTcpEndpoint>();
    }

    std::shared_ptr<ITcpSocket> create_tcp_socket(IIoContext & /*io_context*/) override
    {
        return std::make_shared<MockTcpSocket>();
    }

    std::shared_ptr<IUDPEndpoint> create_udp_endpoint() override { return std::make_shared<MockUdpEndpoint>(); }

    std::shared_ptr<IUDPEndpoint> create_udp_endpoint(asio::ip::address_v4 /*addr*/, uint16_t /*port*/) override
    {
        return std::make_shared<MockUdpEndpoint>();
    }

    std::shared_ptr<IUdpSocket> create_udp_socket(IIoContext & /*io_context*/) override
    {
        // Similar implementation for UDP socket mocks
        return std::make_shared<MockUdpSocket>();
    }

    std::shared_ptr<IResolver> create_resolver(IIoContext & /*io_context*/) override
    {
        return std::make_shared<MockResolver>();
    }

    std::shared_ptr<ITimer> create_timer(IIoContext & /*io_context*/) override { return std::make_shared<MockTimer>(); }
};
