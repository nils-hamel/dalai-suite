/*
 *  dalai-suite - vision
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2019 DHLAB, EPFL
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

    # include "dalai-vision.hpp"

/*
    source - constructor/destructor methods
 */

    dl_vision_t::dl_vision_t( le_real_t const dl_span )

        : vs_window( NULL )
        , vs_context( 0 )
        , vs_execute( true )
        , vs_width( 0 )
        , vs_height( 0 )
        , vs_init_x( 0 )
        , vs_init_y( 0 )
        , vs_dist_z( -dl_span )

    {

        /* display mode variable */
        SDL_DisplayMode dl_display;

        /* initialise and check video */
        if ( SDL_Init( SDL_INIT_VIDEO ) < 0 ) {

            /* send message */
            throw( LC_ERROR_CONTEXT );

        }

        /* retrieve display mode */
        if ( SDL_GetCurrentDisplayMode( 0, & dl_display ) < 0 ) {

            /* send message */
            throw( LC_ERROR_CONTEXT );

        } else {

            /* push screen resolution */
            vs_width  = dl_display.w;
            vs_height = dl_display.h;

        }

        /* double buffer */
        SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );

        /* frame buffer size */
        SDL_GL_SetAttribute( SDL_GL_RED_SIZE  , 8 );
        SDL_GL_SetAttribute( SDL_GL_GREEN_SIZE, 8 );
        SDL_GL_SetAttribute( SDL_GL_BLUE_SIZE , 8 );
        SDL_GL_SetAttribute( SDL_GL_ALPHA_SIZE, 8 );

        /* depth buffer size */
        SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 24 );

        /* create window */
        vs_window = SDL_CreateWindow(
            "dalai-suite - dalai-vision",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            vs_width,
            vs_height,
            SDL_WINDOW_FULLSCREEN | SDL_WINDOW_OPENGL
        );

        /* check window */
        if ( vs_window == NULL ) {

            /* send message */
            throw( LC_ERROR_CONTEXT );

        }

        /* create and check context */
        if ( ( vs_context = SDL_GL_CreateContext( vs_window ) ) == NULL ) {

            /* send message */
            throw( LC_ERROR_CONTEXT );

        }

        /* clear color */
        glClearColor( 0.00, 0.02, 0.05, 0.0 );

        /* clear depth */
        glClearDepth( 1.0 );

        /* depth test */
        glEnable( GL_DEPTH_TEST );

        /* blend function */
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

        /* shade model */
        glShadeModel( GL_SMOOTH );

        /* face culling */
        glCullFace( GL_BACK );

        /* enable material color */
        glEnable( GL_COLOR_MATERIAL );

        /* model lighting configuration */
        vs_set_light();

    }

    dl_vision_t::~dl_vision_t() {

        /* delete context */
        SDL_GL_DeleteContext( vs_context );

        /* delete window */
        SDL_DestroyWindow( vs_window );

        /* terminate video */
        SDL_Quit();

    }

/*
    source - accessor methods
 */

    le_size_t dl_vision_t::vs_get_width( le_void_t ) {

        /* return screen width */
        return( vs_width );

    }

    le_size_t dl_vision_t::vs_get_height( le_void_t ) {

        /* return scree height */
        return( vs_height );

    }

/*
    source - mutator methods
 */

    le_void_t dl_vision_t::vs_set_projection( dl_model_t & dl_model ) {

        /* opengl viewport */
        glViewport( 0.0, 0.0, vs_width, vs_height );

        /* opengl matrix mode */
        glMatrixMode( GL_PROJECTION );

        /* reset matrix coefficients */
        glLoadIdentity();

        /* compute matrix coefficients */
        gluPerspective( 45.0, double( vs_width ) / double( vs_height ), dl_model.ml_get_span() * 0.01, dl_model.ml_get_span() * 10.0 );

    }

    le_void_t dl_vision_t::vs_set_light( le_void_t ) {

        /* array variable */
        GLfloat dl_array[4] = { 0.0, 0.0, 0.0, 1.0 };

        /* enable light */
        glEnable( GL_LIGHT0 );

        /* assign ambient color */
        dl_array[0] = 0.4;
        dl_array[1] = 0.4;
        dl_array[2] = 0.4;

        /* ambient light */
        glLightfv( GL_LIGHT0, GL_AMBIENT, dl_array );

        /* assign diffuse color */
        dl_array[0] = 1.0;
        dl_array[1] = 1.0;
        dl_array[2] = 1.0;

        /* diffuse light */
        glLightfv( GL_LIGHT0, GL_DIFFUSE, dl_array );

        /* light direction */
        dl_array[0] = 0.0;
        dl_array[1] = 0.0;
        dl_array[2] = 1.0;
        dl_array[3] = 0.0;

        /* light direction */
        glLightfv( GL_LIGHT0, GL_POSITION, dl_array );

    }

    le_void_t dl_vision_t::vs_set_center( le_size_t const dl_click_x, le_size_t const dl_click_y, dl_arcball_t & dl_arcball, dl_model_t & dl_model ) {

        /* modelview variable */
        GLdouble dl_modelview[16];

        /* projection variable */
        GLdouble dl_projection[16];

        /* viewport variable */
        GLint dl_viewport[4];

        /* position variable */
        double dl_px( 0.0 );
        double dl_py( 0.0 );
        double dl_pz( 0.0 );

        /* depth variable */
        float dl_depth( 0.0 );

        /* retrieve click depth */
        glReadPixels( dl_click_x, vs_height - dl_click_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, & dl_depth );

        /* check depth value */
        if ( dl_depth < 1.0 ) {

            /* retrieve viewport vector */
            glGetIntegerv( GL_VIEWPORT, dl_viewport );

            /* retrieve projection matrix */
            glGetDoublev( GL_PROJECTION_MATRIX, dl_projection );

            /* matrix mode */
            glMatrixMode( GL_MODELVIEW );

            /* matrix to identity */
            glLoadIdentity();

            /* view translations */
            glTranslatef( 0, 0, vs_dist_z );

            /* arcball rotation */
            dl_arcball.ab_get_rotate();

            /* model translation */
            dl_model.ml_get_translation();

            /* retreive modelview matrix */
            glGetDoublev( GL_MODELVIEW_MATRIX, dl_modelview );

            /* compute click 3-position */
            gluUnProject( dl_click_x, vs_height - dl_click_y, dl_depth, dl_modelview, dl_projection, dl_viewport, & dl_px, & dl_py, & dl_pz );

            /* assign modelview center */
            dl_model.ml_set_center( dl_px, dl_py, dl_pz );

        }

    }

/*
    source - execution methods
 */

    le_void_t dl_vision_t::vs_execution( dl_arcball_t & dl_arcball, dl_model_t & dl_model ) {

        /* event variable */
        SDL_Event dl_event;

        /* principale execution loop */
        while ( vs_execute == true ) {

            /* events management */
            while ( SDL_PollEvent( & dl_event ) > 0 ) {

                /* switch on event type */
                switch ( dl_event.type ) {

                    /* event : keydown */
                    case ( SDL_KEYDOWN ) : {

                        /* specialised method */
                        vs_keydown( dl_event.key, dl_arcball, dl_model );

                    } break;

                    /* event : mouse button */
                    case ( SDL_MOUSEBUTTONDOWN ) : {

                        /* specialised method */
                        vs_button( dl_event.button, dl_arcball, dl_model );

                    } break;

                    /* event : mouse motion */
                    case ( SDL_MOUSEMOTION ) : {

                        /* specialised method */
                        vs_motion( dl_event.motion, dl_arcball, dl_model );

                    } break;

                    /* event : mouse wheele */
                    case ( SDL_MOUSEWHEEL ) : {

                        /* specialised method */
                        vs_wheel( dl_event.wheel, dl_arcball, dl_model );

                    } break;

                };

            }

            /* clear color and depth buffer */
            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

            /* matrix mode */
            glMatrixMode( GL_MODELVIEW );

            /* matrix to identity */
            glLoadIdentity();

            /* view translation */
            glTranslatef( 0, 0, vs_dist_z );

            /* arcball rotation */
            dl_arcball.ab_get_rotate();

            /* render frame */
            dl_model.ml_ren_frame();

            /* model translation */
            dl_model.ml_get_translation();

            /* render model */
            dl_model.ml_ren_model();

            /* swap buffers */
            SDL_GL_SwapWindow( vs_window );

        }

    }

/*
    source - event methods
 */

    le_void_t dl_vision_t::vs_keydown( SDL_KeyboardEvent dl_event, dl_arcball_t & dl_arcball, dl_model_t & dl_model ) {

        /* switch on keycode */
        switch ( dl_event.keysym.sym ) {

            /* keycode : [escape] */
            case ( SDLK_ESCAPE ) : {

                /* update state */
                vs_execute = false;

            } break;

            case ( SDLK_1 ) :
            case ( SDLK_2 ) :
            case ( SDLK_3 ) :
            case ( SDLK_4 ) : {

                /* display point size */
                glPointSize( dl_event.keysym.sym - SDLK_1 + 1 );

                /* display line width */
                glLineWidth( dl_event.keysym.sym - SDLK_1 + 1 );

            } break;

            case ( SDLK_TAB ) : {

                /* switch display flag */
                dl_model.ml_set_switch();

            } break;

            case ( SDLK_RETURN ) : {

                /* compute and display intersection */
                dl_model.ml_get_intersection();

            } break;

            case ( SDLK_BACKSPACE ) : {

                /* clear points */
                dl_model.ml_set_clear();

            } break;

            case ( SDLK_q ) : {

                /* select active surface */
                dl_model.ml_set_surface( 0 );

            } break;

            case ( SDLK_w ) : {

                /* select active surface */
                dl_model.ml_set_surface( 1 );

            } break;

            case ( SDLK_e ) : {

                /* select active surface */
                dl_model.ml_set_surface( 2 );

            } break;

            case ( SDLK_a ) : {

                /* surface point auto-push */
                dl_model.ml_set_auto( -1 );

            } break;

            case ( SDLK_s ) : {

                /* surface point auto-push */
                dl_model.ml_set_auto( 0 );

            } break;

            case ( SDLK_d ) : {

                /* surface point auto-push */
                dl_model.ml_set_auto( +1 );

            } break;

            case ( SDLK_p ) : {

                /* update polygon mode */
                glPolygonMode( GL_FRONT_AND_BACK, GL_POINT );

            } break;

            case ( SDLK_o ) : {

                /* update polygon mode */
                glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

            } break;

            case ( SDLK_i ) : {

                /* update polygon mode */
                glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

            } break;

        };

    }

    le_void_t dl_vision_t::vs_button( SDL_MouseButtonEvent dl_event, dl_arcball_t & dl_arcball, dl_model_t & dl_model ) {

        /* push click position */
        vs_init_x = dl_event.x;
        vs_init_y = dl_event.y;

        /* switch on button */
        switch ( dl_event.button ) {

            case ( SDL_BUTTON_LEFT ) : {

                /* check click type */
                if ( dl_event.clicks == 2 ) {

                    /* update model center */
                    vs_set_center( vs_init_x, vs_init_y, dl_arcball, dl_model );

                }

            } break;

            case ( SDL_BUTTON_MIDDLE ) : {

                /* check click type */
                if ( dl_event.clicks == 1 ) {

                    /* push model center */
                    dl_model.ml_set_push();

                }

            } break;

        };

    }

    le_void_t dl_vision_t::vs_motion( SDL_MouseMotionEvent dl_event, dl_arcball_t & dl_arcball, dl_model_t & dl_model ) {

        /* check mouse state */
        if ( ( dl_event.state & SDL_BUTTON_LMASK ) != 0 ) {

            /* update arcball rotation */
            dl_arcball.ab_set_update( vs_init_x, vs_init_y, dl_event.x, dl_event.y );

            /* push motion position */
            vs_init_x = dl_event.x;
            vs_init_y = dl_event.y;

        }

    }

    le_void_t dl_vision_t::vs_wheel( SDL_MouseWheelEvent dl_event, dl_arcball_t & dl_arcball, dl_model_t & dl_model ) {

        /* inertia variable */
        le_real_t dl_inertia( dl_model.ml_get_span() * 0.02 );

        /* check modifier */
        if ( SDL_GetModState() & KMOD_CTRL ) {

            /* inertia correction */
            dl_inertia *= 8;

        } else if ( SDL_GetModState() & KMOD_SHIFT ) {

            /* inertia correction */
            dl_inertia /= 8;

        }

        /* update model distance */
        vs_dist_z += ( ( dl_event.y > 0 ) ? 1.0 : -1.0 ) * dl_inertia;

    }

/*
    source - main function
 */

    int main( int argc, char ** argv ) {

    /* error management */
    try {

        /* model variable */
        dl_model_t dl_model( ( le_char_t * ) lc_read_string( argc, argv, "--uv3", "-i" ) );

        /* vision variable */
        dl_vision_t dl_vision( dl_model.ml_get_span() );

        /* arcball variable */
        dl_arcball_t dl_arcball( dl_vision.vs_get_width(), dl_vision.vs_get_height() );

        /* set projection matrix */
        dl_vision.vs_set_projection( dl_model );

        /* principale execution loop */
        dl_vision.vs_execution( dl_arcball, dl_model );

    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

