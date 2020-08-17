version=''
commithash='9e826e837c36726998c064eca3795f9b33cba21a'
gittag_short=''
gittag_long='9e826e8-dirty'
git_lastmod='Fri, 7 Aug 2020 13:43:17 -0400'
github_url='https://github.com/plasmodic/ecto'

breathe_default_project = 'ecto'
breathe_projects = dict(ecto='/home/willy/ws/build/ecto/doc/../api/xml')

# for debug: this is only if you build everything locally
#ecto_module_url_root = '/home/willy/ws/build/ecto/doc/../../doc/html/'
# for release
ecto_module_url_root = 'http://plasmodic.github.com/'

intersphinx_mapping = {
                       'ectoimagepipeline': (ecto_module_url_root + 'ecto_image_pipeline', None),
                       'ectoopenni': (ecto_module_url_root + 'ecto_openni', None),
                       'ectoopencv': (ecto_module_url_root + 'ecto_opencv', None),
                       'ectopcl': (ecto_module_url_root + 'ecto_pcl', None),
                       'ectoros': (ecto_module_url_root + 'ecto_ros', None),
                       }

programoutput_path = ''.split(';')
