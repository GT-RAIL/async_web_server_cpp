#include <boost/regex.hpp>
#include "async_web_server_cpp/http_request_handler.hpp"

namespace async_web_server_cpp
{

HttpRequestHandlerGroup::HttpRequestHandlerGroup(HttpServerRequestHandler default_handler)
    : default_handler_(default_handler)
{
}

class PathMatcher
{
public:
  explicit PathMatcher(const std::string &path_regex_string)
      : path_regex_(boost::regex(path_regex_string))
  {
  }

  bool operator()(const HttpRequest &request)
  {
    return regex_match(request.path, path_regex_);
  }

private:
  const boost::regex path_regex_;
};

void HttpRequestHandlerGroup::addHandlerForPath(const std::string &path_regex, HttpServerRequestHandler handler)
{
  addHandler(PathMatcher(path_regex), handler);
}

void HttpRequestHandlerGroup::addHandler(HandlerPredicate predicate, HttpServerRequestHandler handler)
{
  handlers_.push_back(std::make_pair(predicate, handler));
}


void HttpRequestHandlerGroup::operator()(const HttpRequest &request, boost::shared_ptr<HttpConnection> connection, const char* begin, const char* end)
{
  for (int i = 0; i < handlers_.size(); ++i)
  {
    std::pair<HandlerPredicate, HttpServerRequestHandler> &handler = handlers_[i];
    if (handler.first(request))
    {
      handler.second(request, connection, begin, end);
      return;
    }
  }
  default_handler_(request, connection, begin, end);
}

}
