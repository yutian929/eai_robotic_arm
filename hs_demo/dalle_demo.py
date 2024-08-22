from openai import OpenAI
import requests

client = OpenAI()

response = client.images.generate(
  model="dall-e-2",
  prompt="画一只小猫，儿童简笔画，黑白，简单一点",
  size="256x256",
  quality="standard",
  n=1,
)

image_url = response.data[0].url
# 发送HTTP请求并下载图片
response = requests.get(image_url)

# 检查响应状态码
if response.status_code == 200:
    # 打开一个文件，并将图片内容写入文件中
    with open('dalle_output.jpg', 'wb') as f:
        f.write(response.content)
    print("图片下载成功，并保存为dalle_output.jpg")
else:
    print("图片下载失败")