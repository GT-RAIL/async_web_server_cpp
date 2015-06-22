#include "async_web_server_cpp/http_server.hpp"
#include "async_web_server_cpp/http_reply.hpp"

using namespace async_web_server_cpp;

int main(int argc, char **argv)
{
  HttpRequestHandlerGroup handler_group(
    HttpReply::stock_reply(HttpReply::not_found));

  handler_group.addHandlerForPath("/response/ok", HttpReply::stock_reply(HttpReply::ok));
  handler_group.addHandlerForPath("/response/created", HttpReply::stock_reply(HttpReply::created));
  handler_group.addHandlerForPath("/response/accepted", HttpReply::stock_reply(HttpReply::accepted));
  handler_group.addHandlerForPath("/response/forbidden", HttpReply::stock_reply(HttpReply::forbidden));
  handler_group.addHandlerForPath("/response/not_found", HttpReply::stock_reply(HttpReply::not_found));
  handler_group.addHandlerForPath("/response/internal_server_error", HttpReply::stock_reply(HttpReply::internal_server_error));

  HttpServer server("0.0.0.0", "9849", handler_group, 1);

  server.run();

  while(true) {
    sleep(1);
  }

  return (0);
}
