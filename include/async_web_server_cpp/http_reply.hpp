#ifndef CPP_WEB_SERVER_HTTP_REPLY_HPP
#define CPP_WEB_SERVER_HTTP_REPLY_HPP

#include <vector>
#include <string>
#include <boost/asio.hpp>
#include "async_web_server_cpp/http_header.hpp"
#include "async_web_server_cpp/http_connection.hpp"
#include "async_web_server_cpp/http_request_handler.hpp"

namespace async_web_server_cpp
{

class ReplyBuilder;

/**
 *  Utility methods for constructing replys
 */
struct HttpReply
{
  enum status_type
  {
    switching_protocols = 101,
    ok = 200,
    created = 201,
    accepted = 202,
    no_content = 204,
    multiple_choices = 300,
    moved_permanently = 301,
    moved_temporarily = 302,
    not_modified = 304,
    bad_request = 400,
    unauthorized = 401,
    forbidden = 403,
    not_found = 404,
    internal_server_error = 500,
    not_implemented = 501,
    bad_gateway = 502,
    service_unavailable = 503
  } status;

  static std::vector<boost::asio::const_buffer> to_buffers(const std::vector<HttpHeader> &headers);

  /**
   * Create a request handler that sends a stock reply based on the stats code
   */
  static HttpServerRequestHandler stock_reply(status_type status);

  /**
   * Create a request handler that sends the contents of a file
   * NOTE: the file is only loaded once when the request handler is created
   */
  static HttpServerRequestHandler from_file(HttpReply::status_type status,
      const std::string& content_type,
      const std::string& filename,
      const std::vector<HttpHeader>& additional_headers = std::vector<HttpHeader>());

  /**
   * Create a request handler that sends a static response
   */
  static HttpServerRequestHandler static_reply(status_type status,
      const std::string& content_type,
      const std::string& content,
      const std::vector<HttpHeader>& additional_headers = std::vector<HttpHeader>());

  /**
   * Create a builder to create and send reply headers
   */
  static ReplyBuilder builder(status_type status);
};

/**
 * Object to build and send reply headers
 */
class ReplyBuilder
{
public:
  ReplyBuilder(HttpReply::status_type status);

  /**
   * Add a header to the reply
   */
  ReplyBuilder &header(const std::string &name, const std::string &value);

  /**
   * Add a header to the reply
   */
  ReplyBuilder &header(const HttpHeader &header);

  /**
   * Add a group of headers to the reply
   */
  ReplyBuilder &headers(const std::vector<HttpHeader> &headers);

  /**
   * Send the headers over the connection
   */
  void write(HttpConnectionPtr connection);

private:
  HttpReply::status_type status_;
  boost::shared_ptr<std::vector<HttpHeader> > headers_;
};


/**
 *  Request Handler that serves a predefined response
 */
class StaticHttpRequestHandler
{
public:
  StaticHttpRequestHandler(HttpReply::status_type status,
                           const std::vector<HttpHeader> &headers,
                           const std::string &content);

  void operator()(const HttpRequest &, boost::shared_ptr<HttpConnection>, const char* begin, const char* end);

private:
  ReplyBuilder reply_builder_;
  const std::string content_string_;
};

}

#endif
