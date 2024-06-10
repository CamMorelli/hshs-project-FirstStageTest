import subprocess

Import("env")

def get_firmware_specifier_build_flag():
    ret = subprocess.run(["git", "describe", "--tags", "--always"], stdout=subprocess.PIPE, text=True)
    git_version = ret.stdout.strip()
    build_flag = "-D GIT_VERSION=\\\"" + git_version + "\\\""
    print ("git version: [" + git_version + "]" )
    return build_flag

env.Append(
    BUILD_FLAGS=[get_firmware_specifier_build_flag()]
)