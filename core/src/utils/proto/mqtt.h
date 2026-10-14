#pragma once

// based on this file: https://github.com/fredilarsen/ReconnectingMqttClient
// Based on the spec http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Toc398718016

#include <inttypes.h>
#include <string>
#include <functional>
#include <string.h>
#include "http.h"
#include "ctm.h"
#include <utils/usleep.h>

namespace net {


    class MQTTClient {
    public:
        // Fixed byte sequences
        const uint8_t CONNECT = 1 << 4,
                CONNACK = 2 << 4,
                PUBLISH = 3 << 4,
                PUBACK = 4 << 4,
                SUBSCRIBE = (8 << 4) | 2, // 0b10000010
        SUBACK = 9 << 4,
                UNSUBSCRIBE = (10 << 4) | 2, // 0b10100010
        UNSUBACK = 11 << 4,
                PINGREQ_2[2] = {12 << 4, 0},
                PINGRESP_2[2] = {13 << 4, 0},
                DISCONNECT_2[2] = {14 << 4, 0},
                CONNECT_7[7] = {0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04}; // MQTT version 4 = 3.1.1

        const uint16_t KEEPALIVE_S = 60, PING_TIMEOUT = 15000;
        const uint32_t READ_TIMEOUT = 10000;

        std::string topic, client_id = "sdr++brown", user, password;
        uint8_t server_ip[4], sub_qos = 1;
        uint16_t port = 1883;

        std::shared_ptr<net::Socket> client;

        bool enabled = true;
        uint16_t msg_id = 1, pubacked_msg_id = 0;
        bool waiting_for_ping = false;
        uint32_t last_packet_in = 0, last_packet_out = 0;
        int8_t last_connect_error = 0x7F; // Unknown
        std::function<void(const char *topic,const uint8_t *payload,uint16_t len)> receive_callback = NULL;
        char topicbuf[1024];
        uint8_t buffer[1024];
        volatile bool last_sub_acked = false, last_pub_acked = false; // With QoS 1 the success of the last SUB or PUB can be checked

        void init_system() {
        }

        void cleanup_system() {
#ifdef _WIN32
            WSACleanup(); // Cleanup Winsock
#endif
        }

        void response_delay() {
        }

        // Portable alternative to itoa which is not available everywhere, Returns chars added.
        uint8_t uint8toa(uint8_t n, char *p) {
            uint8_t a = n / 100, b = (n % 100) / 10, c = n % 10;
            char *p2 = p;
            if (a != 0) *p2++ = '0' + a;
            if (b != 0) *p2++ = '0' + b;
            *p2++ = '0' + c;
            *p2 = 0; // Null-terminate
            return p2 - p;
        }

        // Write a header into the start of the buffer and return the number of bytes written
        uint16_t put_header(const uint8_t header, uint8_t *buf, const uint16_t len) {
            uint16_t l = len;
            uint8_t pos = 0, v;
            buf[pos++] = header;
            do {
                v = l % 0x80;
                l /= 0x80;
                buf[pos++] = l > 0 ? v | 0x80 : v;
            }
            while (l != 0);
            return pos;
        }

        // Write an UTF-8 text into the buffer at the given offset, return number of bytes written
        uint16_t put_string(const char *text, uint16_t len, uint8_t *buf, const uint16_t pos) {
            uint16_t p = pos;
            if (len == 0) return 0;
            buf[p++] = len >> 8;
            buf[p++] = len & 0xFF;
            memcpy(&buf[p], text, len);
            return 2 + len;
        }

        uint16_t put_string(const char *text, uint8_t *buf, const uint16_t pos) {
            return put_string(text, (uint16_t) strlen(text), buf, pos);
        }

        bool send_disconnect() { return write_to_socket(DISCONNECT_2, 2); }

        bool send_pingreq() {
            waiting_for_ping = true;
            return write_to_socket(PINGREQ_2, 2);
        }

        bool send_pingresp() { return write_to_socket(PINGRESP_2, 2); }

        uint32_t inactivity_time() {
            uint32_t in = last_packet_in == 0 ? 1000000000 : (uint32_t) (currentTimeMillis() - last_packet_in),
                    out = last_packet_out == 0 ? 1000000000 : (uint32_t) (currentTimeMillis() - last_packet_out);
            return in < out ? in : out;
        }

        bool write_to_socket(const uint8_t *buf, const uint16_t len) {
            if (client && client->isOpen()) {
                uint16_t remain = len;
                while (remain > 0) {
                    int written = client->send(buf, remain);
                    if (written <= 0) break;
                    remain -= written;
                }
                bool ok = remain == 0;
#ifdef MQTT_DEBUGPRINT
                printf("%u Sent packet %u len %d\n", millis(), buf[0], len);
#endif
                if (ok) last_packet_out = currentTimeMillis();
                return ok;
            }
            return false;
        }

        bool read_from_socket(uint8_t *buf, const uint16_t len, const uint16_t startpos = 0, bool blocking = true) {
            if (client && client->isOpen()) {
                uint16_t remain = len, pos = startpos;
                uint32_t start = currentTimeMillis();
                while (remain > 0) {
                    if (!client || !client->isOpen()) {
                        return false;
                    }
                    int n = client->recv(&buf[pos], remain, true, 1000);
                    if (n == 0 && client->isOpen()) {
                        // eagain
                        send_ping_if_needed();
                        if (onIdle) {
                            onIdle();
                        }
                        continue;
                    }
#if defined(PJON_ESP) && defined(ESP32)
                    if (n == -1) n = 0; // ESP32 returns -1 if nothing there, not only if connection broken
#else
                    if (n == -1 || !client->isOpen()) break; // Normal behavior, -1 = error, connection broken
#endif
                    if (n == 0 && (uint32_t) (currentTimeMillis() - start) > READ_TIMEOUT) break;
                    if (!blocking && n == 0 && remain == len) break; // available() sometimes gives false positive, exit if nothing
                    if (n == 0) usleep(1000);
                    remain -= n;
                }
                return remain == 0;
            }
            return false;
        }

        bool read_packet_from_socket(uint8_t *buf, const uint16_t bufsize, uint16_t &packet_len,
                                     uint16_t &payload_len, bool blocking = true) {
            packet_len = payload_len = buf[0] = 0;
            if (!read_from_socket(buf, 2, 0, blocking)) return false;
            payload_len = buf[1] & 0x7F; // Remove upper bit
            uint16_t pos = 2;
            // Read length of payload
            uint32_t scaling = 1;
            while (buf[pos - 1] & 0x80) { // If uppermost bit is set
                if (!read_from_socket(buf, 1, pos++)) return false;
                scaling *= 0x80;
                payload_len += (buf[pos - 1] & 0x7F) * scaling;
            }
            // Read payload
            if (payload_len > 0) {
                if (pos + payload_len >= bufsize) return false; // Too big
                if (!read_from_socket(buf, payload_len, pos)) return false;
                pos += payload_len;
                buf[pos] = 0; // Null-terminate after payload to simplify usage if text
            }
            packet_len = pos;
            last_packet_in = currentTimeMillis();
#ifdef MQTT_DEBUGPRINT
            printf("%u Received packet %u len %d\n", millis(), buf[0], packet_len);
#endif
            return true;
        }

        bool socket_connect(std::string host, uint16_t port) {
            gotPong = 0;
            try {
                client = net::connect(host, port);
            } catch (const std::exception &e) {
                return false;
            }

            // Compose packet
            uint16_t len = 0, payloadsize = (uint16_t) (10 + (client_id.length() + 2)
                                                        + (user.length() > 0 ? user.length() + 2 : 0)
                                                        + (password.length() > 0 ? password.length() + 2 : 0));
            len += put_header(CONNECT, buffer, payloadsize);
            memcpy(&buffer[len], CONNECT_7, 7);
            len += 7;
            uint8_t flags = 0x02; // Clean session, No will
            if (user.length() > 0) flags |= password.length() > 0 ? 0x80 | (0x80 >> 1) : 0x80;
            buffer[len++] = flags;
            buffer[len++] = KEEPALIVE_S >> 8;
            buffer[len++] = KEEPALIVE_S & 0xFF;
            len += put_string(client_id.c_str(), buffer, len);
            len += put_string(user.c_str(), buffer, len);
            len += put_string(password.c_str(), buffer, len);
            last_connect_error = 0x7F;
            last_packet_in = currentTimeMillis();
            if (write_to_socket(buffer, len)) {
                response_delay();
                uint16_t packet_len, payload_len;
                if (read_packet_from_socket(buffer, sizeof buffer, packet_len, payload_len)) {
                    if (packet_len == 4 && buffer[0] == CONNACK) {
                        if (buffer[3] == 0) {
                            // Subscribe if a topic has been set
                            if (topic.length() > 0) {
                                bool ok = send_subscribe(topic.c_str(), sub_qos);
                                if (!ok) stop();
                            }
                            return client->isOpen();
                        } else last_connect_error = buffer[3]; // Got an error code
                    }
                }
            }
            return false;
        }

        void handle_publish(const uint8_t *buf, const uint16_t packet_len, const uint16_t payload_len) {
            if (receive_callback) {
                uint16_t pos = packet_len - payload_len;
                uint8_t s0 = buf[pos++], s1 = buf[pos++];
                uint16_t textlen = (((int)s0) << 8) | s1;
                if (textlen > payload_len) {
                    flog::info("{} {} {} {} {}", buf[0],buf[1],buf[2],buf[3], buf[4]);
                }
                memcpy(topicbuf, &buf[pos], textlen);
                topicbuf[textlen] = 0; // Null terminator
                pos += textlen;

                if (buf[0] & 0b00000110) { // QOS1 or QOS2
                    uint8_t sendbuf[4];
                    sendbuf[0] = PUBACK;
                    sendbuf[1] = 2;
                    sendbuf[2] = buf[pos++]; // message id MSB
                    sendbuf[3] = buf[pos++]; // message id LSB
                    write_to_socket(sendbuf, 4);
                }
                receive_callback(topicbuf, &buf[pos], packet_len - pos);
            }
        }

        void send_ping_if_needed() {
            if (inactivity_time() > PING_TIMEOUT) {
                if (waiting_for_ping) {
                    stop();
                    start();
                }
//                else
                send_pingreq();
            }
        }

        const char *next_topic(const char *p) {
            while (*p && *p != ',') p++;
            return p;
        }

        bool send_subscribe(const char *topic, const uint8_t qos, bool unsubscribe = false) {
            last_sub_acked = false;
            if (client && client->isOpen()) {
                // Pre-scan to find total payload length
                uint16_t payload_len = 2; // packet identifier
                const char *p = topic, *p2 = p;
                while (*p && (p2 = next_topic(p))) { // Find next comma or final null-terminator
                    payload_len += ((uint16_t) (p2 - p) + 2) + (unsubscribe ? 0 : 1);
                    p = *p2 ? p2 + 1 : p2; // Skip comma if several topics are listed
                }
                // Add header and packet identifier
                uint16_t len = put_header(unsubscribe ? UNSUBSCRIBE : SUBSCRIBE, buffer, payload_len);
                if (++msg_id == 0) msg_id++; // Avoid 0
                buffer[len++] = msg_id >> 8;
                buffer[len++] = msg_id & 0xFF;
                // Add each topic
                p = topic;
                while (*p && (p2 = next_topic(p))) {
                    payload_len += ((uint16_t) (p2 - p) + 2) + (unsubscribe ? 0 : 1);
                    len += put_string(p, (uint16_t) (p2 - p), buffer, len);
                    if (!unsubscribe) buffer[len++] = qos;
                    p = *p2 ? p2 + 1 : p2; // Skip comma if several topics are listed
                }
                return write_to_socket(buffer, len);
            }
            return false;
        }

        void handle_suback(const uint8_t *buf, const uint16_t packet_len, const uint16_t payload_len, bool unsubscribe) {
            if (packet_len == (unsubscribe ? 4 : 5) && buf[0] == (unsubscribe ? UNSUBACK : SUBACK)) {
                uint16_t mess_id = (buf[2] << 8) | buf[3];
                if (!unsubscribe && buf[4] > 2) {
                    flog::info("MQTT Subscribe error");
                    return; // Return code indicates failure
                }
                if (mess_id == msg_id) {
                    flog::info("MQTT Subscribe OK");
                    last_sub_acked = true;
                }
            }
        }

        void handle_puback(const uint8_t *buf, const uint16_t packet_len, const uint16_t payload_len) {
            if (packet_len == 4 && buf[0] == PUBACK) {
                pubacked_msg_id = (buf[2] << 8) | buf[3];
                if (pubacked_msg_id == msg_id) last_pub_acked = true;
            }
        }

    public:
        MQTTClient() {}

        MQTTClient(const uint8_t server_ip[4], const uint16_t server_port, const char *client_id) {
            set_address(server_ip, server_port, client_id);
        }

        ~MQTTClient() { stop(); }

        void set_address(const uint8_t server_ip[4], const uint16_t server_port, const char *client_id) {
            memcpy(this->server_ip, server_ip, 4);
            port = server_port;
            this->client_id = client_id;
            start();
        }

        bool publish(const char *topic, const uint8_t *payload, const uint16_t payloadlen, const bool retain, const uint8_t qos = 0) {
            last_pub_acked = false;
            if (client && client->isOpen()) {
                uint16_t total = ((uint16_t) strlen(topic) + 2) + (qos > 0 ? 2 : 0) + payloadlen,
                        len = put_header(PUBLISH, buffer, total);
                if (retain) buffer[0] |= 1;
                buffer[0] |= qos << 1; // Add QOS into second or third bit
                len += put_string(topic, buffer, len);
                if (qos > 0) {
                    if (++msg_id == 0) msg_id++;
                    buffer[len++] = msg_id >> 8;
                    buffer[len++] = msg_id & 0xFF;
                }
                memcpy(&buffer[len], payload, payloadlen);
                len += payloadlen;
                bool ok = write_to_socket(buffer, len);
                if (qos == 0) last_pub_acked = ok;
                return ok;
            }
            return false;
        }

        // Text-only version for convenience
        bool publish(const char *topic, const char *payload, const bool retain, const uint8_t qos = 0) {
            return publish(topic, (const uint8_t *) payload, (uint16_t) strlen(payload), retain, qos);
        }

        // When subscribing, multiple topics can be listed separated by comma
        bool subscribe(const std::string &topic, const uint8_t qos = 1) {
            if (this->topic != "") unsubscribe();
            this->topic = topic;
            sub_qos = qos;
            return send_subscribe(this->topic.c_str(), qos, false);
        }

        bool unsubscribe() {
            bool ok = send_subscribe(this->topic.c_str(), 0, true);
            this->topic = "";
            return ok;
        }

        long long gotPong = 0;

        bool update() {
            if (client && client->isOpen()) {
                send_ping_if_needed();
                uint16_t packet_len, payload_len;
                if (read_packet_from_socket(buffer, sizeof buffer, packet_len, payload_len, false)) {
                    if (packet_len > 1) {
                        if ((buffer[0] & PUBLISH) == PUBLISH) handle_publish(buffer, packet_len, payload_len);
                        else if (buffer[0] == PUBACK) handle_puback(buffer, packet_len, payload_len);
                        else if (buffer[0] == SUBACK) handle_suback(buffer, packet_len, payload_len, false);
                        else if (buffer[0] == UNSUBACK) handle_suback(buffer, packet_len, payload_len, true);
                        else if (buffer[0] == PINGREQ_2[0]) send_pingresp();
                        else if (buffer[0] == PINGRESP_2[0]) {
                            gotPong = currentTimeMillis();
                            waiting_for_ping = false;
                        }
#ifdef MQTT_DEBUGPRINT
                        else printf("%u Received UNKNOWN packet %u len %d\n", millis(), buffer[0], packet_len);
#endif
                    }
                    return true;
                } else {
                    if (client) {
                        client->close();
                        client.reset();
                    }
                    return false;
                }
            }
            return client && client->isOpen();
        }

        void start() {
            enabled = true;
            last_packet_in = last_packet_out = currentTimeMillis();
            init_system();
        }

        void stop() {
//            if (this->topic.length() > 0) send_subscribe(this->topic.c_str(), true);
            send_disconnect();
            if (client) {
                client->close();
                client.reset();
            }
            enabled = false;
            cleanup_system();
        }

        // The subscribe call and the publish calls With QoS 1 will return true or false depending on whether
        // the message was written, not whether an ACK was received. This can be checked here, and it may be set
        // not immediately but some time later. Other packets may be received before the ACK arrives.
        bool was_last_sub_acked() const { return last_sub_acked; }

        bool was_last_pub_acked() const { return last_pub_acked; }

        uint16_t last_pub_msgid() const { return msg_id; }

        uint16_t last_puback_msgid() const { return pubacked_msg_id; }

        bool wait_for_puback(uint16_t timeout_ms = 100) {
            uint32_t start = currentTimeMillis();
            while (!was_last_pub_acked() && ((uint32_t) (currentTimeMillis() - start) < timeout_ms)) update();
            return was_last_pub_acked();
        }

        char *topic_buf() { return topicbuf; } // Allow temporary access for composing outgoing topic, saving memory
        std::function<void()> onIdle;
    };

}