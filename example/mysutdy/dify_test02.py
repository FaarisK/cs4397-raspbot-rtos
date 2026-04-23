import requests
import json,os
import mimetypes


api_key = "app-iTlHn9ETR4LqqVFBBbAMZkum" # 你的API密钥
base_url = "http://localhost" 
user_id = "abc-123" # 用户标识
image_path = "./rec.jpg" #图片路径


def upload_image():
    url = f"{base_url}/v1/files/upload"
    # 确定文件类型 Determine file type
    filename = os.path.basename(image_path)
    file_extension = os.path.splitext(filename)[1].lower().lstrip('.')
    mime_type = mimetypes.guess_type(image_path)[0] or "application/octet-stream"

    # 支持的图片类型映射 Supported image type mapping
    image_types = {
        'png': 'image/png',
        'jpeg': 'image/jpeg',
        'jpg': 'image/jpeg',
        'webp': 'image/webp',
        'gif': 'image/gif'
    }

    # 验证是否为支持的图片类型 Verify if it is a supported image type
    if file_extension not in image_types:
        raise ValueError(f"Unsupported image type: {file_extension}. Only supported: png, jpeg, jpg, webp, gif")

    # 设置正确的 MIME 类型 Set the correct MIME type
    if mime_type != image_types[file_extension]:
        mime_type = image_types[file_extension]

    # 准备请求头 Prepare request header
    headers = {
        "Authorization": f"Bearer {api_key}"
    }
 
    # 准备表单数据 Prepare form data
    files = {
        'file': (filename, open(image_path, 'rb'), mime_type)
    }

    data = {
        'user': user_id
    }

    try:
        response = requests.post(url, headers=headers, files=files, data=data)
        
        # 检查响应
        if response.status_code in (200, 201):
            result = response.json()
            print("File uploaded successfully!")
            # print(f"文件 ID: {result['id']}")
            # print(f"文件名: {result['name']}")
            # print(f"文件大小: {result['size']} 字节")
            # print(f"MIME 类型: {result['mime_type']}")
            return result['id']  # 返回文件 ID 用于后续操作 Return file ID for subsequent operations
        else:
            print(f"File upload failed, status code: {response.status_code}")
            print(f"error message: {response.text}")
            return None

    except Exception as e:
        print(f"Request error occurred: {e}")
        return None

    

 
def run_dify_workflow(strQ="Go forward for 3 seconds, describe what's in the picture?"): 
    
    #上传图片
    file_id = upload_image()
       
    # 配置参数
    url = f"{base_url}/v1/chat-messages" 
    # 请求头
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json;charset=UTF-8",
        "Accept-Charset": "UTF-8"
    }
 
    # 请求体
    payload = {
        "inputs": {},
        "query":strQ,
        "response_mode": "blocking",
        "user": user_id,
        "files": [
        {
            "type": "image",
            "transfer_method": "local_file",
            "upload_file_id": file_id  # 这里使用上传后获得的文件 ID
        }
    ]
    }
 
    try:
        # 发送POST请求
        response = requests.post(
            url,
            headers=headers,
            json=payload,
            stream=False  # 重要：启用流式响应
        )
        #print(f"请求成功，状态码: {response.status_code}")

        # 检查响应状态
        if response.status_code != 200:
            print(f"Request failed, status code: {response.status_code}")
            print(f"error: {response.text}")
            return

               
        if response.status_code == 200:
            answer = response.json()["answer"]
            #print(answer)
            if answer:
                return answer
            else:
                return ""
        else:  
            return 0
        

    except requests.exceptions.RequestException as e:
        print(f"请求发生错误: {e}")
        
    except Exception as e:
        print(e)
        return 0

 
 
if __name__ == "__main__":
    aaa=run_dify_workflow()
    print(aaa)