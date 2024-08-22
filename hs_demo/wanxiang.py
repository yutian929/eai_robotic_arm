from http import HTTPStatus
from urllib.parse import urlparse, unquote
from pathlib import PurePosixPath
import requests
import dashscope


def simple_call():
    prompt = '画一只小猫,要求是：手绘，线条，黑白，简单一点，儿童画风格'
    rsp = dashscope.ImageSynthesis.call(model=dashscope.ImageSynthesis.Models.wanx_v1,
                              prompt=prompt,
                              n=1,
                              size='1024*1024')
    if rsp.status_code == HTTPStatus.OK:
        print(rsp.output)
        print(rsp.usage)
        # save file to current directory
        for result in rsp.output.results:
            file_name = PurePosixPath(unquote(urlparse(result.url).path)).parts[-1]
            with open('./%s' % file_name, 'wb+') as f:
                f.write(requests.get(result.url).content)
    else:
        print('Failed, status_code: %s, code: %s, message: %s' %
              (rsp.status_code, rsp.code, rsp.message))


if __name__ == '__main__':
    simple_call()