#include "cpp_epuck/EpuckPackets.hpp"
#include "cpp_epuck/UDPComms.hpp"
#include "mock_NetworkFactory.hpp"
#include <cpp_epuck/RobotCommsModel.hpp>
#include <gtest/gtest.h>
#include <iterator>
#include <memory>
#include <utility>

// // File is just an entrypoint to run the unit tests from
// int main(int argc, char **argv)
// {
//     testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }

class TestRobotCommsModel : public ::testing::Test
{
public:
    TestRobotCommsModel()
    {
        const robot_id_type robot_id                 = 0x123;
        host_size_string manager_host                = "manager_host";
        const uint16_t manager_port                  = 0x456;
        host_size_string robot_comms_host            = "robot_comms_hst";
        const uint16_t robot_comms_request_port      = 0x789;
        host_size_string robot_knowledge_host        = "robot_knldg_hst";
        const uint16_t robot_knowledge_exchange_port = 0xabc;
        auto network_factory                         = std::make_shared<MockNetworkFactory>();

        model = std::make_shared<RobotCommsModel<UDPKnowledgeServer, UDPKnowledgeClient>>(
            robot_id, manager_host, manager_port, robot_comms_host, robot_comms_request_port, robot_knowledge_host,
            robot_knowledge_exchange_port, network_factory);
    }

    std::shared_ptr<RobotCommsModel<UDPKnowledgeServer, UDPKnowledgeClient>> model;
};

TEST_F(TestRobotCommsModel, TestConstructor)
{
    uint64_t expected_robot_id = 0x123;
    std::string expected_manager_host("manager_host");
    uint64_t expected_manager_port = 0x456;
    std::string expected_robot_comms_host("robot_comms_hst");
    uint64_t expected_robot_comms_request_port = 0x789;
    std::string expected_robot_knowledge_host("robot_knldg_hst");
    uint64_t expected_robot_knowledge_exchange_port = 0xabc;

    // Check that the constructor sets the values correctly, could pick up issues with robot_id_type
    // or host_size_string changes
    EXPECT_EQ(expected_robot_id, model->robot_id);
    EXPECT_STREQ(expected_manager_host.c_str(), model->manager_host.c_str());
    EXPECT_EQ(expected_manager_port, model->manager_port);
    EXPECT_STREQ(expected_robot_comms_host.c_str(), model->robot_comms_host.c_str());
    EXPECT_EQ(expected_robot_comms_request_port, model->robot_comms_request_port);
    EXPECT_STREQ(expected_robot_knowledge_host.c_str(), model->robot_knowledge_host.c_str());
    EXPECT_EQ(expected_robot_knowledge_exchange_port, model->robot_knowledge_exchange_port);
}

TEST_F(TestRobotCommsModel, TestGetSeq)
{
    // Check that the seq is set correctly in the constructor
    uint16_t expected_seq = 2;
    EXPECT_EQ(expected_seq, model->get_seq());

    // Check that the seq is incremented correctly
    for (expected_seq = 3; expected_seq < 1024; expected_seq++)
    {
        EXPECT_EQ(expected_seq, model->get_seq());
    }
}

TEST_F(TestRobotCommsModel, TestGetKnownIds)
{
    // Check that the known ids are set correctly in the constructor
    auto expected_known_ids = std::vector<EpuckKnowledgeRecord>(
        {EpuckKnowledgeRecord{model->robot_id, model->get_centroid(), model->get_boundary(), 1}});

    for (auto known_id = model->known_ids_begin(); known_id != model->known_ids_end(); known_id++)
    {
        auto idx               = std::distance(model->known_ids_begin(), known_id);
        auto expected_known_id = expected_known_ids[idx];
        EXPECT_EQ(expected_known_id.robot_id, (*known_id).robot_id);
        EXPECT_EQ(expected_known_id.centroid, (*known_id).centroid);
        EXPECT_EQ(expected_known_id.boundary, (*known_id).boundary);
        EXPECT_EQ(expected_known_id.seq, (*known_id).seq);
    }
}

TEST_F(TestRobotCommsModel, TestKnownIdsSize)
{
    // Check that the known ids size is set correctly in the constructor
    size_t expected_known_ids_size = 1;
    EXPECT_EQ(expected_known_ids_size, model->known_ids_size());
}

TEST_F(TestRobotCommsModel, TestInsertKnownIds) {}

TEST_F(TestRobotCommsModel, TestCreateKnowledgePacket)
{
    // Check that the knowledge packet is created correctly
    auto expected_knowledge_packet =
        EpuckKnowledgePacket{0x22,
                             model->robot_id,
                             2,
                             1,
                             {EpuckKnowledgeRecord{model->robot_id, model->get_centroid(), model->get_boundary(), 2}}};

    auto knowledge_packet = model->create_knowledge_packet();
    EXPECT_EQ(expected_knowledge_packet.id, knowledge_packet.id);
    EXPECT_EQ(expected_knowledge_packet.robot_id, knowledge_packet.robot_id);
    EXPECT_EQ(expected_knowledge_packet.seq, knowledge_packet.seq);
    EXPECT_EQ(expected_knowledge_packet.N, knowledge_packet.N);
    EXPECT_EQ(expected_knowledge_packet.known_ids[0].robot_id, knowledge_packet.known_ids[0].robot_id);
    EXPECT_EQ(expected_knowledge_packet.known_ids[0].centroid, knowledge_packet.known_ids[0].centroid);
    EXPECT_EQ(expected_knowledge_packet.known_ids[0].boundary, knowledge_packet.known_ids[0].boundary);
    EXPECT_EQ(expected_knowledge_packet.known_ids[0].seq, knowledge_packet.known_ids[0].seq);
}

TEST_F(TestRobotCommsModel, TestSetCentroid)
{
    // Check that the centroid is set correctly
    auto expected_centroid = Centroid{1.0, 2.0, 3.0};
    model->set_centroid(expected_centroid);
    EXPECT_EQ(expected_centroid, model->get_centroid());
}

TEST_F(TestRobotCommsModel, TestSetBoundary)
{
    // Check that the boundary is set correctly
    std::array<float, MAX_BOUNDARY_X_POINTS> x_points{};
    std::array<float, MAX_BOUNDARY_Y_POINTS> y_points{};
    std::array<float, MAX_BOUNDARY_Z_POINTS> z_points{};
    uint32_t idx = 0;

    // clang-format off
    // Disable -Wtype-limits warning for this block, since some of the MAX_POINTS may be zero
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wtype-limits"
    for (idx = 0; idx < MAX_BOUNDARY_X_POINTS; idx++)
    {
        x_points.at(idx) = static_cast<float>(idx);
    }
    for (idx = 0; idx < MAX_BOUNDARY_Y_POINTS; idx++)
    {
        y_points.at(idx) = static_cast<float>(idx + MAX_BOUNDARY_X_POINTS);
    }
    for (idx = 0; idx < MAX_BOUNDARY_Z_POINTS; idx++)
    {
        z_points.at(idx) = static_cast<float>(idx + MAX_BOUNDARY_X_POINTS + MAX_BOUNDARY_Y_POINTS);
    }
    #pragma GCC diagnostic pop
    // clang-format on

    auto expected_boundary = Boundary{x_points, y_points, z_points};
    model->set_boundary(expected_boundary);
    EXPECT_EQ(expected_boundary, model->get_boundary());
}

TEST_F(TestRobotCommsModel, TestGetCentroid)
{
    // Check that the centroid is set correctly in the constructor
    auto expected_centroid = Centroid{0.0, 0.0, 0.0};
    EXPECT_EQ(expected_centroid, model->get_centroid());
}

TEST_F(TestRobotCommsModel, TestGetBoundary)
{
    // Check that the boundary is set correctly in the constructor
    auto expected_boundary = Boundary{{}, {}, {}};
    EXPECT_EQ(expected_boundary, model->get_boundary());
}
